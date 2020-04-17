#include "tracerimpl.h"
#include "tracer.h"
#include "ioimpl.h"
#include "moduleimpl.h"
#include "context.h"
#include "verilogwriter.h"

using namespace ch::internal;

#define NUM_TRACES 100

struct vcd_signal_compare_t {
  bool operator()(ioportimpl* lhs, ioportimpl* rhs) const{
    auto lhs_pos  = lhs->name().find_last_of('/');
    auto rhs_pos  = rhs->name().find_last_of('/');
    auto lhs_path = (lhs_pos != std::string::npos) ? lhs->name().substr(0, lhs_pos) : "";
    auto rhs_path = (rhs_pos != std::string::npos) ? rhs->name().substr(0, rhs_pos) : "";
    if (lhs_path < rhs_path)
      return true;
    if (lhs_path > rhs_path)
      return false;
    auto lhs_name = (lhs_pos != std::string::npos) ? lhs->name().substr(lhs_pos+1) : lhs->name();
    auto rhs_name = (rhs_pos != std::string::npos) ? rhs->name().substr(rhs_pos+1) : rhs->name();
    return (lhs_name < rhs_name);
  }
};

auto remove_path = [](const std::string& path) {
  auto pos = path.find('/');
  return (pos != std::string::npos) ? path.substr(pos+1) : path;
};

tracerimpl::tracerimpl(const std::vector<device_base>& devices)
  : simulatorimpl(devices)
  , trace_width_(0)
  , ticks_(0)
  , trace_head_(nullptr)
  , trace_tail_(nullptr)
  , num_traces_(0)
  , is_single_context_(1 == contexts_.size() && 0 == contexts_.back()->modules().size()) {
  if ((platform::self().cflags() & ch_flags::verbose_tracing) != 0) {
    verbose_tracing_ = true;
  }
}

tracerimpl::~tracerimpl() {
  auto block = trace_head_;
  while (block) {
    auto next = block->next;
    block->~trace_block_t();
    ::operator delete [](block);
    block = next;
  }
}

void tracerimpl::initialize() {
  //--
  simulatorimpl::initialize();

  //--
  auto add_signal = [&](ioportimpl* node) {
    signals_.emplace_back(node);
    return node->size();
  };

  //--
  auto trace_width = 0;
  auto clk = eval_ctx_->sys_clk();
  if (clk) {
    trace_width += add_signal(clk);
  }

  auto reset = eval_ctx_->sys_reset();
  if (reset) {
    trace_width += add_signal(reset);
  }

  for (auto node : eval_ctx_->inputs()) {
    auto signal = reinterpret_cast<ioportimpl*>(node);
    if (signal == clk || signal == reset)
      continue;
    trace_width += add_signal(signal);
  }

  for (auto node : eval_ctx_->outputs()) {
    auto signal = reinterpret_cast<ioportimpl*>(node);
    trace_width += add_signal(signal);
  }

  for (auto node : eval_ctx_->taps()) {
    auto signal = reinterpret_cast<ioportimpl*>(node);
    trace_width += add_signal(signal);
  }

  trace_width_ = trace_width + signals_.size();
  prev_values_.resize(signals_.size());
  valid_mask_.resize(signals_.size());
}

void tracerimpl::eval() {
  // advance simulation
  simulatorimpl::eval();

  // allocate new trace block
  auto block_width = NUM_TRACES * trace_width_;
  if (nullptr == trace_tail_
   || (trace_tail_->size + trace_width_) > block_width) {
    this->allocate_trace(block_width);
  }

  // log trace data
  valid_mask_.reset();
  auto dst_block = trace_tail_->data;
  auto dst_offset = trace_tail_->size;
  dst_offset += valid_mask_.size();
  for (uint32_t i = 0, n = signals_.size(); i < n; ++i) {
    auto value = signals_[i]->value();
    auto& prev = prev_values_.at(i);
    if (prev.first) {
      if (0 == bv_cmp(reinterpret_cast<const block_type*>(prev.first),
                      prev.second, value->words(), 0, value->size()))
        continue;
    }
    prev.first = dst_block;
    prev.second = dst_offset;
    bv_copy(reinterpret_cast<block_type*>(dst_block),
            dst_offset, value->words(), 0, value->size());
    dst_offset += value->size();    
    valid_mask_[i] = true;
  }

  // set valid mask
  bv_copy(dst_block, trace_tail_->size, valid_mask_.words(), 0, valid_mask_.size());

  // updsate offset
  trace_tail_->size = dst_offset;

  ++ticks_;
}

void tracerimpl::allocate_trace(uint32_t block_width) {
  auto block_size = (bitwidth_v<block_t> / 8) * ceildiv(block_width, bitwidth_v<block_t>);
  auto buf = new uint8_t[sizeof(trace_block_t) + block_size]();
  auto data = reinterpret_cast<block_t*>(buf + sizeof(trace_block_t));
  auto trace_block = new (buf) trace_block_t(data);
  if (nullptr == trace_head_) {
    trace_head_ = trace_block;
  }
  if (trace_tail_) {
    trace_tail_->next = trace_block;
  }
  trace_tail_ = trace_block;
  ++num_traces_;
}

void tracerimpl::toText(std::ofstream& out) const {
  //--
  auto get_signal_name = [&](ioportimpl* node) {
    if (!is_single_context_ && 1 == contexts_.size()) {
      return remove_path(node->name());
    }
    return node->name();
  };

  uint32_t t = 0;
  auto mask_width = valid_mask_.size();
  auto indices_width = std::to_string(ticks_).length();

  std::vector<std::pair<block_t*, uint32_t>> 
      prev_values(signals_.size(), std::make_pair<block_t*, uint32_t>(nullptr, 0));

  auto trace_block = trace_head_;
  while (trace_block) {
    auto src_block = trace_block->data;
    auto src_width = trace_block->size;
    uint32_t src_offset = 0;
    while (src_offset < src_width) {
      uint32_t mask_offset = src_offset;
      src_offset += mask_width;
      out << std::setw(indices_width) << t << ":";
      auto_separator sep(",");
      for (uint32_t i = 0, n = signals_.size(); i < n; ++i) {
        auto signal = signals_[i];
        auto signal_type = signal->type();
        auto signal_size = signal->size();
        auto signal_name = get_signal_name(signal);
        bool valid = bv_get(src_block, mask_offset + i);
        if (valid) {
          auto value = get_value(src_block, signal_size, src_offset);
          out << sep << " " << signal_name << "=" << value;
          if (type_input != signal_type) {
            auto& prev = prev_values.at(i);
            prev.first = src_block;
            prev.second = src_offset;
          }
          src_offset += signal_size;
        } else {
          if (type_input != signal_type) {
            auto& prev = prev_values.at(i);
            assert(prev.first);
            auto value = get_value(prev.first, signal_size, prev.second);
            out << sep << " " << signal_name << "=" << value;
          }
        }
      }
      out << std::endl;
      ++t;
    }
    trace_block = trace_block->next;
  }
}

void tracerimpl::toVCD(std::ofstream& out) const {  
  dup_tracker<std::string> dup_mod_names;
  std::list<std::string> mod_stack;

  std::set<ioportimpl*, vcd_signal_compare_t> sorted_signals;
  for (auto node : signals_) {
    sorted_signals.emplace(node);
  }

  // log trace header
  out << "$timescale 1 ns $end" << std::endl;

  for (auto node : sorted_signals) {
    if (is_single_context_) {
      out << "$var reg " << node->size() << ' ' << node->id() << ' '
          << identifier_from_string(node->name()) << " $end" << std::endl;
    } else {      
      auto path = split(node->name(), '/');
      auto name = path.back(); // get name
      path.pop_back(); // remove name
      if (path.empty()) {
        path.push_back("sys");
      }

      auto path_it = path.begin();
      auto stack_it = mod_stack.begin();
      while (path_it != path.end()
          && stack_it != mod_stack.end()) {
        if (*stack_it != *path_it) {
          auto del_it = stack_it;
          while (del_it != mod_stack.end()) {
            out << "$upscope $end" << std::endl;
            del_it = mod_stack.erase(del_it);
          }
          break;
        }
        ++path_it;
        ++stack_it;
      }

      while (path_it != path.end()) {
        auto mod = *path_it++;
        auto mod_name = mod;
        auto num_dups = dup_mod_names.insert(mod_name);
        if (num_dups) {
          mod_name = stringf("%s_%ld", mod_name.c_str(), num_dups);
        }
        out << "$scope module " << mod_name << " $end" << std::endl;
        mod_stack.push_back(mod);
      }
      out << "$var reg " << node->size() << ' ' << node->id() << ' '
          << identifier_from_string(name) << " $end" << std::endl;
    }
  }

  while (!mod_stack.empty()) {
    out << "$upscope $end" << std::endl;
    mod_stack.pop_back();
  }  
  out << "$enddefinitions $end" << std::endl;

  // log trace data
  uint32_t t = 0;
  auto mask_width = valid_mask_.size();

  auto trace_block = trace_head_;
  while (trace_block) {
    auto src_block = trace_block->data;
    auto src_width = trace_block->size;
    uint32_t src_offset = 0;
    while (src_offset < src_width) {
      uint32_t mask_offset = src_offset;
      src_offset += mask_width;
      bool new_trace = false;
      for (uint32_t i = 0, n = signals_.size(); i < n; ++i) {
        bool valid = bv_get(src_block, mask_offset + i);
        if (valid) {
          if (!new_trace) {
            out << '#' << t << std::endl;
            new_trace = true;
          }
          auto signal = signals_[i];
          auto signal_size = signal->size();
          auto value = get_value(src_block, signal_size, src_offset);
          src_offset += signal_size;
          if (signal_size > 1) {
            out << 'b';
          }
          size_t q = 0;
          for (auto it = value.rbegin(), end = value.rend(); it != end;) {
            ++q;
            out << (*it++ ? '1' : '0');            
          }
          if (signal_size > 1)
            out << ' ';
          out << signal->id() << std::endl;
        }
      }
      if (new_trace)
        out << std::endl;
      ++t;
    }
    trace_block = trace_block->next;
  }
}

void tracerimpl::toVerilog(std::ofstream& out,
                           const std::string& moduleFileName,
                           bool passthru) const {
  //--
  auto netlist_name = [&](lnodeimpl* node)->std::string {
    std::stringstream ss;
    verilogwriter::print_node_name(ss, node);
    return ss.str();
  };

  //--
  auto get_tap_path = [&](tapimpl* node) {    
    if (is_single_context_) {
      auto sname = netlist_name(node);
      return stringf("%s_%d.%s", node->ctx()->name().c_str(), node->ctx()->id(), sname.c_str());
    } else {
      auto pos   = node->name().find_last_of('/');
      auto path  = node->name().substr(0, pos);
      std::replace(path.begin(), path.end(), '/', '.');
      auto sname = identifier_from_string(node->name().substr(pos+1));
      return stringf("%s.%s", path.c_str(), sname.c_str());;
    }
  };

  //--
  auto get_signal_name = [&](ioportimpl* node) {
    auto path = node->name();
    if (!is_single_context_) {
      auto sname = remove_path(path);
      if (sname == "clk" || sname == "reset")
        return sname;
      if (1 == contexts_.size()) {
        path = sname;
      }
    }
    path = identifier_from_string(path);
    return path;
  };

  //--
  auto find_signal_name = [&](ioportimpl* node) {
    if ((node->name() == "clk") || (node->name() == "reset"))
      return node->name();
    auto name = node->name();
    if (!is_single_context_) {
      name = stringf("%s_%d/%s", node->ctx()->name().c_str(), node->ctx()->id(), name.c_str());
    }
    for (auto signal : signals_) {      
      if (signal->name() == name)
        return get_signal_name(signal);
    }
    std::abort();
    return std::string();
  };

  //--
  auto print_type = [](std::ostream& out, ioimpl* node) {
    out << (type_input == node->type() ? "reg" : "wire");
    if (node->size() > 1)
      out << "[" << (node->size() - 1) << ":0]";
  };

  //--
  auto print_value = [](std::ostream& out, const bv_t& value) {
    if (value.size() > 4 || value.word(0) > 9) {
      out << value.size() << "'h";
    }

    auto oldflags = out.flags();
    out.setf(std::ios_base::hex, std::ios_base::basefield);

    bool skip_zeros = true;
    uint32_t word = 0;
    auto size = value.size();

    for (auto it = value.begin() + (size - 1); size;) {
      word = (word << 0x1) | *it--;
      if (0 == (--size & 0x3)) {
        if (0 == size || (word != 0 ) || !skip_zeros) {
          out << word;
          skip_zeros = false;
        }
        word = 0;
      }
    }
    if (0 != (size & 0x3)) {
      out << word;
    }
    out.flags(oldflags);
  };

  //--
  auto print_module = [&](std::ostream& out, context* ctx) {
    auto_separator sep(", ");
    out << ctx->name() << " " << ctx->name() << "_" << ctx->id() << "(";
    for (auto node : ctx->inputs()) {
      auto input = reinterpret_cast<inputimpl*>(node);
      out << sep << '.' << identifier_from_string(input->name()) << "(" << find_signal_name(input) << ")";
    }
    for (auto node : ctx->outputs()) {
      auto output = reinterpret_cast<outputimpl*>(node);
      out << sep << '.' << identifier_from_string(output->name()) << "(" << find_signal_name(output) << ")";
    }
    out << ");" << std::endl;
    return true;
  };

  // log header
  out << "`timescale 1ns/1ns" << std::endl;
  out << "`include \"" << moduleFileName << "\"" << std::endl << std::endl;
  out << "`define check(x, y) if ((x == y) !== 1)"
         " if ((x == y) === 0) $error(\"x=%h, expected=%h\", x, y);"
         " else $warning(\"x=%h, expected=%h\", x, y)" << std::endl << std::endl;
  out << "module testbench();" << std::endl << std::endl;

  {
    auto_indent indent(out);
    int has_clock = 0;
    int has_taps = 0;

    // declare signals
    for (auto signal : signals_) {
      print_type(out, signal);
      auto name = get_signal_name(signal);
      out << " " << name << ";" << std::endl;
      has_clock |= (name == "clk");
      has_taps |= (type_tap == signal->type());
    }
    out << std::endl;

    // declare modules
    for (auto ctx : contexts_) {
      print_module(out, ctx);
      out << std::endl;
    }

    if (has_taps) {
      for (auto signal : signals_) {
        if (type_tap != signal->type())
          continue;
        out << "assign " << get_signal_name(signal) << " = "
            << get_tap_path(reinterpret_cast<tapimpl*>(signal)) << ";" << std::endl;
      }
      out << std::endl;
    }

    // declare clock process
    if (has_clock) {
      out << "always begin" << std::endl;
      {
        auto_indent indent1(out);
        out << "#1 clk = !clk;" << std::endl;
      }
      out << "end" << std::endl << std::endl;
    }

    // declare simulation process
    out << "initial begin" << std::endl;
    {
      auto_indent indent1(out);

      uint64_t tc = 0, tp = 0;
      auto mask_width = valid_mask_.size();

      std::vector<std::pair<block_t*, uint32_t>> 
          prev_values(signals_.size(), std::make_pair<block_t*, uint32_t>(nullptr, 0));

      auto trace_block = trace_head_;
      while (trace_block) {
        auto src_block = trace_block->data;
        auto src_width = trace_block->size;
        uint32_t src_offset = 0;
        while (src_offset < src_width) {
          uint32_t mask_offset = src_offset;
          auto in_offset = mask_offset + mask_width;
          auto out_offset = mask_offset + mask_width;
          bool in_trace = false;
          bool out_trace = false;
          {
            for (uint32_t i = 0, n = signals_.size(); i < n; ++i) {
              bool valid = bv_get(src_block, mask_offset + i);
              if (valid) {
                auto signal = signals_[i];
                auto signal_type = signal->type();
                auto signal_size = signal->size();
                auto signal_name = get_signal_name(signal);
                if ((type_input != signal_type) // is not an input signal
                 || (tc != 0 && signal_name == "clk")) {  // is not clk signal initialization
                  in_offset += signal_size;
                  continue;
                }
                if (!in_trace) {
                  out << "#" << (tc - tp);
                  tp = tc;
                  in_trace = true;
                }
                auto value = get_value(src_block, signal_size, in_offset);
                in_offset += signal_size;
                out << " " << signal_name << "=";
                print_value(out, value);
                out << ";";
              }
            }
            if (in_trace) {
              out << std::endl;
            }
          }

          {
            for (uint32_t i = 0, n = signals_.size(); i < n; ++i) {
              auto signal = signals_[i];
              auto signal_type = signal->type();
              auto signal_size = signal->size();
              auto signal_name = get_signal_name(signal);
              bool valid = bv_get(src_block, mask_offset + i);
              if (valid) {
                if (type_input == signal_type) {
                  out_offset += signal_size;
                  continue;
                }

                auto& prev = prev_values.at(i);
                prev.first = src_block;
                prev.second = out_offset;

                if (passthru
                 && (tc + 1) < ticks_) {
                  out_offset += signal_size;
                  continue; // skip signals value validation
                }

                if (!out_trace) {
                  out << "#" << (tc - tp + 1);
                  tp = tc + 1;
                  out_trace = true;
                }

                auto value = get_value(src_block, signal_size, out_offset);
                out_offset += signal_size;
                out << " `check(" << signal_name << ", ";
                print_value(out, value);
                out << ");";
              } else {
                if (type_input == signal_type)
                  continue;

                if (passthru
                 && (tc + 1) < ticks_)
                  continue;

                if (!out_trace) {
                  out << "#" << (tc - tp  + 1);
                  tp = tc + 1;
                  out_trace = true;
                }

                auto& prev = prev_values.at(i);
                assert(prev.first);
                auto value = get_value(prev.first, signal_size, prev.second);
                out << " `check(" << signal_name << ", ";
                print_value(out, value);
                out << ");";
              }
            }
            if (out_trace) {
              out << std::endl;
            }
          }

          src_offset = in_offset;
          ++tc;
        }
        trace_block = trace_block->next;
      }
      out << "#1 $finish;" << std::endl;
    }
    out << "end" << std::endl << std::endl;
  }

  // log footer
  out << "endmodule" << std::endl;
}

void tracerimpl::toVerilator(std::ofstream& out,
                             const std::string& moduleTypeName) const {
  //--
  auto get_signal_name = [&](ioportimpl* node) {
    auto path = node->name();
    if (!is_single_context_ && 1 == contexts_.size()) {
      path = remove_path(path);
    }
    path = identifier_from_string(path);
    return path;
  };

  //--
  auto print_value = [](std::ostream& out,
                        const bv_t& value,
                        uint32_t size = 0,
                        uint32_t offset = 0) {
    if (size > 4 || value.word(0) > 9) {
      out << "0x";
    }

    bool skip_zeros = true;
    if (0 == size) {
      size = value.size();
    }

    auto oldflags = out.flags();
    out.setf(std::ios_base::hex, std::ios_base::basefield);

    uint32_t word(0);
    for (auto it = value.begin() + offset + (size - 1); size;) {
      word = (word << 0x1) | *it--;
      if (0 == (--size & 0x3)) {
        if (0 == size || (word != 0 ) || !skip_zeros) {
          out << word;
          skip_zeros = false;
        }
        word = 0;
      }
    }
    if (0 != (size & 0x3)) {
      out << word;
    }
    out.flags(oldflags);
  };

  //--
  auto print_input = [&](const bv_t& value, const std::string& signal_name, uint32_t signal_size) {
    if (signal_size > 64) {
      out << "vl_setw(sim_->" << signal_name;
      for (uint32_t j = 0; j < signal_size;) {
        out << ", ";
        auto s = (j+32 <= signal_size) ? 32 : (signal_size-j);
        print_value(out, value, s, j);        
        j += s;
      }
      out << ");" << std::endl;
    } else {
      out << "sim_->" << signal_name << "=";
      print_value(out, value);
      out << ";" << std::endl;
    }
  };

  //--
  auto print_output = [&](const bv_t& value, const std::string& signal_name, uint32_t signal_size) {
    if (signal_size > 64) {
      out << "CHECK(vl_cmpw(sim_->" << signal_name;
      for (uint32_t j = 0; j < signal_size;) {
        out << ", ";
        auto s = (j+32 <= signal_size) ? 32 : (signal_size-j);
        print_value(out, value, s, j);
        j += s;
      }
      out << ") == 0);" << std::endl;
    } else {
      out << "CHECK(sim_->" << signal_name << " == ";
      print_value(out, value);
      out << ");" << std::endl;
    }
  };

  if (contexts_.size() > 1) {
    throw std::invalid_argument("multiple devices not supported!");
  }

  uint64_t tc = 0;
  auto mask_width = valid_mask_.size();

  std::vector<std::pair<block_t*, uint32_t>> 
      prev_values(signals_.size(), std::make_pair<block_t*, uint32_t>(nullptr, 0));

  out << "#pragma once" << std::endl;
  out << "#include <cash/eda/verilator/vl_simulator.h>" << std::endl;
  out << "#define CHECK(x) do { if (!(x)) { std::cout << \"FAILED: \" << #x << std::endl; } } while (false)" << std::endl;
  out << "class vl_testbench {" << std::endl;
  out << "public:" << std::endl;
  out << "bool eval(uint64_t tick) {" << std::endl;
  out << "switch (tick) {" << std::endl;

  auto trace_block = trace_head_;
  while (trace_block) {
    auto src_block = trace_block->data;
    auto src_width = trace_block->size;
    uint32_t src_offset = 0;
    while (src_offset < src_width) {
      uint32_t mask_offset = src_offset;
      auto in_offset = mask_offset + mask_width;
      auto out_offset = mask_offset + mask_width;
      bool trace_enable = false;
      {
        for (uint32_t i = 0, n = signals_.size(); i < n; ++i) {
          bool valid = bv_get(src_block, mask_offset + i);
          if (valid) {
            auto signal = signals_[i];
            auto signal_type = signal->type();
            auto signal_size = signal->size();
            auto signal_name = get_signal_name(signal);
            if ((type_input != signal_type)
              || signal_name == "clk"
              || signal_name == "reset") {
              in_offset += signal_size;
              continue;
            }
            if (!trace_enable) {
              out << "case " << tc << ":" << std::endl;
              trace_enable = true;
            }
            auto value = get_value(src_block, signal_size, in_offset);
            in_offset += signal_size;
            print_input(value, signal_name, signal_size);
          }
        }
      }

      {
        for (uint32_t i = 0, n = signals_.size(); i < n; ++i) {
          auto signal = signals_[i];
          auto signal_type = signal->type();
          auto signal_size = signal->size();
          auto signal_name = get_signal_name(signal);
          bool valid = bv_get(src_block, mask_offset + i);
          if (valid) {
            if (type_input == signal_type) {
              out_offset += signal_size;
              continue;
            }
            auto& prev = prev_values.at(i);
            prev.first = src_block;
            prev.second = out_offset;
            if ((tc + 1) < ticks_) {
              out_offset += signal_size;
              continue;
            }
            if (!trace_enable) {
              out << "case " << (tc + 1) << ":" << std::endl;
              trace_enable = true;
            }
            auto value = get_value(src_block, signal_size, out_offset);
            out_offset += signal_size;
            print_output(value, signal_name, signal_size);
          } else {
            if (type_input == signal_type)
              continue;
            if ((tc + 1) < ticks_)
              continue;
            if (!trace_enable) {
              out << "case " << (tc + 1) << ":" << std::endl;
              trace_enable = true;
            }
            auto& prev = prev_values.at(i);
            assert(prev.first);
            auto value = get_value(prev.first, signal_size, prev.second);
            print_output(value, signal_name, signal_size);
          }
        }
      }
      if (trace_enable) {
        out << "break;" << std::endl;
      }
      src_offset = in_offset;
      ++tc;
    }
    trace_block = trace_block->next;
  }

  out << "}" << std::endl;
  out << "return (tick < " << ticks_ << ");" << std::endl;
  out << "}" << std::endl;
  out << "auto reset(uint64_t tick) { return sim_.reset(tick); }" << std::endl;
  out << "auto step(uint64_t tick) { return sim_.step(tick, 2); }" << std::endl;
  out << "private:" << std::endl;
  out << "vl_simulator<" << moduleTypeName << "> sim_;" << std::endl;
  out << "};" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////

void tracerimpl::toSystemC(std::ofstream& out,
                           const std::string& moduleTypeName) const {
  //--
  auto get_signal_name = [&](ioportimpl* node) {
    auto path = node->name();
    if (!is_single_context_ && 1 == contexts_.size()) {
      path = remove_path(path);
    }
    path = identifier_from_string(path);
    return path;
  };

  //--
  auto print_value = [](std::ostream& out,
                        const bv_t& value,
                        uint32_t size = 0,
                        uint32_t offset = 0) {
    if (size > 4 || value.word(0) > 9) {
      out << "0x";
    }

    bool skip_zeros = true;
    if (0 == size) {
      size = value.size();
    }

    auto oldflags = out.flags();
    out.setf(std::ios_base::hex, std::ios_base::basefield);

    uint32_t word(0);
    for (auto it = value.begin() + offset + (size - 1); size;) {
      word = (word << 0x1) | *it--;
      if (0 == (--size & 0x3)) {
        if (0 == size || (word != 0 ) || !skip_zeros) {
          out << word;
          skip_zeros = false;
        }
        word = 0;
      }
    }
    if (0 != (size & 0x3)) {
      out << word;
    }
    out.flags(oldflags);
  };

  //--
  auto print_input = [&](const bv_t& value, const std::string& signal_name, uint32_t signal_size) {
    if (signal_size > 64) {
      out << "sc_setw(" << signal_name << "_";
      for (uint32_t j = 0; j < signal_size;) {
        out << ", ";
        auto s = (j+32 <= signal_size) ? 32 : (signal_size-j);
        print_value(out, value, s, j);        
        j += s;
      }
      out << ");" << std::endl;
    } else {
      out << signal_name << "_ = ";
      print_value(out, value);
      out << ";" << std::endl;
    }
  };

  //--
  auto print_output = [&](const bv_t& value, const std::string& signal_name, uint32_t signal_size) {
    if (signal_size > 64) {
      out << "CHECK(sc_cmpw(" << signal_name<< "_";
      for (uint32_t j = 0; j < signal_size;) {
        out << ", ";
        auto s = (j+32 <= signal_size) ? 32 : (signal_size-j);
        print_value(out, value, s, j);
        j += s;
      }
      out << ") == 0);" << std::endl;
    } else {
      out << "CHECK(" << signal_name << "_.read() == ";
      print_value(out, value);
      out << ");" << std::endl;
    }
  };

  if (contexts_.size() > 1) {
    throw std::invalid_argument("multiple devices not supported!");
  }

  uint64_t tc = 0;
  auto mask_width = valid_mask_.size();

  std::vector<std::pair<block_t*, uint32_t>> 
      prev_values(signals_.size(), std::make_pair<block_t*, uint32_t>(nullptr, 0));

  out << "#pragma once" << std::endl;
  out << "#include <cash/eda/verilator/sc_simulator.h>" << std::endl;
  out << "#define CHECK(x) do { if (!(x)) { std::cout << \"FAILED: \" << #x << std::endl; } } while (false)" << std::endl;
  out << "class sc_testbench {" << std::endl;
  out << "public:" << std::endl;
  out << "sc_testbench() {" << std::endl;

  for (auto signal : signals_) {
    if (signal->name() == "clk" || signal->name() == "reset")
      continue;
    auto signal_name = get_signal_name(signal);
    out << "sim_->" << signal_name << "(" << signal_name << "_);" << std::endl;
  }

  out << "#if VM_TRACE" << std::endl;
  out << "auto tfp = sim->create_trace(\""<< moduleTypeName <<"\")" << std::endl;
  for (auto signal : signals_) {
    if (signal->name() == "clk" || signal->name() == "reset")
      continue;
    auto signal_name = get_signal_name(signal);
    out << "sc_trace(tfp, " << signal_name << "_, \"" << signal_name << "\");" << std::endl;
  }
  out << "#endif" << std::endl;
  out << "}" << std::endl;
  out << "bool eval(uint64_t tick) {" << std::endl;
  out << "switch (tick) {" << std::endl;
  auto trace_block = trace_head_;
  while (trace_block) {
    auto src_block = trace_block->data;
    auto src_width = trace_block->size;
    uint32_t src_offset = 0;
    while (src_offset < src_width) {
      uint32_t mask_offset = src_offset;
      auto in_offset = mask_offset + mask_width;
      auto out_offset = mask_offset + mask_width;
      bool trace_enable = false;
      {
        for (uint32_t i = 0, n = signals_.size(); i < n; ++i) {
          bool valid = bv_get(src_block, mask_offset + i);
          if (valid) {
            auto signal = signals_[i];
            auto signal_type = signal->type();
            auto signal_size = signal->size();
            auto signal_name = get_signal_name(signal);
            if ((type_input != signal_type)
             || signal_name == "clk"
             || signal_name == "reset") {
              in_offset += signal_size;
              continue;
            }
            if (!trace_enable) {
              out << "case " << tc << ":" << std::endl;
              trace_enable = true;
            }
            auto value = get_value(src_block, signal_size, in_offset);
            in_offset += signal_size;
            print_input(value, signal_name, signal_size);
          }
        }
      }

      {
        for (uint32_t i = 0, n = signals_.size(); i < n; ++i) {
          auto signal = signals_[i];
          auto signal_type = signal->type();
          auto signal_size = signal->size();
          auto signal_name = get_signal_name(signal);
          bool valid = bv_get(src_block, mask_offset + i);
          if (valid) {
            if (type_input == signal_type) {
              out_offset += signal_size;
              continue;
            }
            auto& prev = prev_values.at(i);
            prev.first = src_block;
            prev.second = out_offset;
            if ((tc + 1) < ticks_) {
              out_offset += signal_size;
              continue;
            }
            if (!trace_enable) {
              out << "case " << (tc + 1) << ":" << std::endl;
              trace_enable = true;
            }
            auto value = get_value(src_block, signal_size, out_offset);
            out_offset += signal_size;
            print_output(value, signal_name, signal_size);
          } else {
            if (type_input == signal_type)
              continue;
            if ((tc + 1) < ticks_)
              continue;
            if (!trace_enable) {
              out << "case " << (tc + 1) << ":" << std::endl;
              trace_enable = true;
            }
            auto& prev = prev_values.at(i);
            assert(prev.first);
            auto value = get_value(prev.first, signal_size, prev.second);
            print_output(value, signal_name, signal_size);
          }
        }
      }
      if (trace_enable) {
        out << "break;" << std::endl;
      }
      src_offset = in_offset;
      ++tc;
    }
    trace_block = trace_block->next;
  }

  out << "}" << std::endl;
  out << "return (tick < " << ticks_ << ");" << std::endl;
  out << "}" << std::endl;
  out << "auto reset(uint64_t tick) { return sim_.reset(tick); }" << std::endl;
  out << "auto step(uint64_t tick) { return sim_.step(tick, 2); }" << std::endl;
  out << "private:" << std::endl;
  out << "sc_simulator<" << moduleTypeName << "> sim_;" << std::endl;

  for (auto signal : signals_) {
    if (signal->name() == "clk" || signal->name() == "reset")
      continue;
    auto signal_size = signal->size();
    auto signal_name = get_signal_name(signal);
    out << "sc_signal<";
    if (signal_size == 1) {
      out << "bool";
    } else if (signal_size <= 32) {
      out << "uint32_t";
    } else if (signal_size <= 64) {
      out << "uint64_t";
    } else {
      out << "sc_bv<" << signal_size << ">";
    }
    out << "> " << signal_name << "_;" << std::endl;
  }
  out << "};" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////

void tracerimpl::toVPI_c(std::ofstream& out) const {
  //--
  auto get_signal_name = [&](ioportimpl* node) {
    auto path = node->name();
    if (!is_single_context_ && 1 == contexts_.size()) {
      path = remove_path(path);
    }
    path = identifier_from_string(path);
    return path;
  };

  //--
  auto print_value = [](std::ostream& out,
                        const bv_t& value,
                        uint32_t size = 0,
                        uint32_t offset = 0) {
    if (size > 4 || value.word(0) > 9) {
      out << "0x";
    }

    bool skip_zeros = true;
    if (0 == size) {
      size = value.size();
    }

    auto oldflags = out.flags();
    out.setf(std::ios_base::hex, std::ios_base::basefield);

    uint32_t word(0);
    for (auto it = value.begin() + offset + (size - 1); size;) {
      word = (word << 0x1) | *it--;
      if (0 == (--size & 0x3)) {
        if (0 == size || (word != 0 ) || !skip_zeros) {
          out << word;
          skip_zeros = false;
        }
        word = 0;
      }
    }
    if (0 != (size & 0x3)) {
      out << word;
    }
    out.flags(oldflags);
  };

  //--
  auto print_input = [&](const bv_t& value, const std::string& signal_name, uint32_t signal_size) {
    if (signal_size > 32) {
      out << "    vpi_setw(s_" << signal_name << ", {";
      for (uint32_t j = 0; j < signal_size;) {
        if (j) out << ", ";
        auto s = (j+32 <= signal_size) ? 32 : (signal_size-j);
        print_value(out, value, s, j);        
        j += s;
      }
      out << "});" << std::endl;
    } else {
      out << "    vpi_seti(s_" << signal_name << ", ";
      print_value(out, value);        
      out << ");" << std::endl;
    }
  };

  //--
  auto print_output = [&](const bv_t& value, const std::string& signal_name, uint32_t signal_size) {
    if (signal_size > 32) {
      out << "    vpi_cmpw(s_" << signal_name << ", {";
      for (uint32_t j = 0; j < signal_size;) {
        if (j) out << ", ";
        auto s = (j+32 <= signal_size) ? 32 : (signal_size-j);
        print_value(out, value, s, j);        
        j += s;
      }
      out << "});" << std::endl;
    } else {
      out << "    vpi_cmpi(s_" << signal_name << ", ";
      print_value(out, value);        
      out << ");" << std::endl;
    }
  };

  if (contexts_.size() > 1) {
    throw std::invalid_argument("multiple devices not supported!");
  }

  uint64_t tc = 0;
  auto mask_width = valid_mask_.size();

  std::vector<std::pair<block_t*, uint32_t>> 
      prev_values(signals_.size(), std::make_pair<block_t*, uint32_t>(nullptr, 0));

  out << "#include <iostream>\n"
         "#include <vector>\n"
         "#include <chrono>\n"
         "#include <vpi_user.h>\n" 
      << std::endl;

  out << "static void vpi_setw(vpiHandle sig, std::initializer_list<unsigned> values) {\n"
         "  static std::vector<s_vpi_vecval> buf;\n"
         "  buf.resize(values.size());\n"
         "  auto bi = buf.begin();\n"
         "  for (auto vi = values.begin(), ve = values.end(); vi != ve; ++vi, ++bi) {\n"
         "    bi->aval = *vi;\n"
         "  }\n"
         "  s_vpi_value value_s;\n"
         "  value_s.format = vpiVectorVal;\n"
         "  value_s.value.vector = buf.data();\n"
         "  vpi_put_value(sig, &value_s, NULL, vpiNoDelay);\n"
         "}\n" 
      << std::endl;
      
  out << "static void vpi_seti(vpiHandle sig, unsigned value) {\n"
         "  s_vpi_value value_s;\n"
         "  value_s.format = vpiIntVal;\n"
         "  value_s.value.integer = value;\n"
         "  vpi_put_value(sig, &value_s, NULL, vpiNoDelay);\n"
         "}\n" 
      << std::endl;

  out << "static bool vpi_cmpw(vpiHandle sig, std::initializer_list<unsigned> values) {\n"
         "  static std::vector<s_vpi_vecval> buf;\n"
         "  buf.resize(values.size());  \n"
         "  s_vpi_value value_s;\n"
         "  value_s.format = vpiVectorVal;\n"
         "  value_s.value.vector = buf.data();\n"
         "  vpi_get_value(sig, &value_s);\n"
         "  auto bi = buf.begin();\n"
         "  for (auto vi = values.begin(), ve = values.end(); vi != ve; ++vi, ++bi) {\n"
         "    if (unsigned(bi->aval) != *vi) return false;\n"
         "  }\n"
         "  return true;\n"
         "}\n"
      << std::endl;

  out << "static bool vpi_cmpi(vpiHandle sig, unsigned value) {\n"
         "  s_vpi_value value_s;\n"
         "  value_s.format = vpiIntVal;\n"
         "  value_s.value.integer = value;\n"
        "  vpi_get_value(sig, &value_s);\n"
        "  return (unsigned(value_s.value.integer) == value);\n"
        "}\n"
      << std::endl;

  out << "bool initialized = false;\n"
         "std::chrono::time_point<std::chrono::system_clock> start_time;\n"
         "double overhead = 0;\n" 
      << std::endl;

  for (auto signal : signals_) {
    auto signal_name = get_signal_name(signal);
    out << "vpiHandle s_" << signal_name << ";\n";
  }
  out << std::endl;

  out << "static PLI_INT32 eval_callback(p_cb_data cb_data) {\n"
         "  static long int ticks = 0;\n"
         "  if (!initialized) {\n"
         "    start_time = std::chrono::system_clock::now();\n"
         "    initialized = true;\n"
         "  }\n"
         "  auto t0 = std::chrono::system_clock::now();\n"
      << std::endl;

  out << "  switch (ticks) {\n";

  auto trace_block = trace_head_;
  while (trace_block) {
    auto src_block = trace_block->data;
    auto src_width = trace_block->size;
    uint32_t src_offset = 0;
    while (src_offset < src_width) {
      uint32_t mask_offset = src_offset;
      auto in_offset = mask_offset + mask_width;
      auto out_offset = mask_offset + mask_width;
      bool trace_enable = false;
      {
        for (uint32_t i = 0, n = signals_.size(); i < n; ++i) {
          bool valid = bv_get(src_block, mask_offset + i);
          if (valid) {
            auto signal = signals_[i];
            auto signal_type = signal->type();
            auto signal_size = signal->size();
            auto signal_name = get_signal_name(signal);
            if ((type_input != signal_type)
              || signal_name == "clk"
              || signal_name == "reset") {
              in_offset += signal_size;
              continue;
            }
            if (!trace_enable) {
              out << "  case " << tc << ":\n";
              trace_enable = true;
            }
            auto value = get_value(src_block, signal_size, in_offset);
            in_offset += signal_size;
            print_input(value, signal_name, signal_size);
          }
        }
      }

      {
        for (uint32_t i = 0, n = signals_.size(); i < n; ++i) {
          auto signal = signals_[i];
          auto signal_type = signal->type();
          auto signal_size = signal->size();
          auto signal_name = get_signal_name(signal);
          bool valid = bv_get(src_block, mask_offset + i);
          if (valid) {
            if (type_input == signal_type) {
              out_offset += signal_size;
              continue;
            }
            auto& prev = prev_values.at(i);
            prev.first = src_block;
            prev.second = out_offset;
            if ((tc + 1) < ticks_) {
              out_offset += signal_size;
              continue;
            }
            if (!trace_enable) {
              out << "  case " << (tc + 1) << ":" << std::endl;
              trace_enable = true;
            }
            auto value = get_value(src_block, signal_size, out_offset);
            out_offset += signal_size;
            print_output(value, signal_name, signal_size);
          } else {
            if (type_input == signal_type)
              continue;
            if ((tc + 1) < ticks_)
              continue;
            if (!trace_enable) {
              out << "  case " << (tc + 1) << ":" << std::endl;
              trace_enable = true;
            }
            auto& prev = prev_values.at(i);
            assert(prev.first);
            auto value = get_value(prev.first, signal_size, prev.second);
            print_output(value, signal_name, signal_size);
          }
        }
      }

      if (trace_enable) {
        out << "    break;\n";
      }
      src_offset = in_offset;
      ++tc;
    }
    trace_block = trace_block->next;
  }

  out << "  }\n" << std::endl;
  
  out << "  auto t1 = std::chrono::system_clock::now();\n"
         "  overhead += std::chrono::duration<double, std::milli>(t1 - t0).count();\n"
         "  if (++ticks == " << ticks_ << ") {\n"
         "    auto end_time = std::chrono::system_clock::now();\n"
         "    double elapsed_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();\n"
         "    std::cout << \"Elapsed Time: \" << elapsed_time << \"ms\" << std::endl;\n"
         "    std::cout << \"Kernel Time: \" << (elapsed_time - overhead)  << \"ms\" << std::endl;\n"
         "    std::cout << \"overhead Time: \" << overhead << \"ms\" << std::endl;\n"
         "    vpi_control(vpiFinish);\n"
         "  }\n"
	       "  return 0;\n"
         "}\n" << std::endl;

  out << "static PLI_INT32 register_callbacks(char *user_data) {\n";

  for (auto signal : signals_) {
    auto signal_name = get_signal_name(signal);
    out << "  s_" << signal_name << " = vpi_handle_by_name(\"testbench." << signal_name << "\", NULL);\n";
  }
  out << std::endl;
         
  out << "  s_vpi_value value_s;\n"
         "  s_vpi_time time_s;\n"
         "  s_cb_data cb_data_s;\n"
         "\n"
         "  time_s.type = vpiSuppressTime;\n"
         "  value_s.format = vpiIntVal;\n"
         "\n"
         "  cb_data_s.user_data = NULL;\n"
         "  cb_data_s.reason = cbValueChange;\n"
         "  cb_data_s.cb_rtn = eval_callback;\n"
         "  cb_data_s.time = &time_s;\n"
         "  cb_data_s.value = &value_s;\n"
         "\n"         
         "  cb_data_s.obj  = s_clk;\n"
         "  vpi_register_cb(&cb_data_s);\n"
         "  return 0;\n"
         "}\n" << std::endl;

  out << "static void register_verilog_functions() {\n"
         "  s_vpi_systf_data tf;\n"
         "  tf.type = vpiSysTask;\n"
         "  tf.tfname = \"$eval_callback\";\n"
         "  tf.calltf = register_callbacks;\n"
         "  tf.compiletf = NULL;\n"
         "  tf.sizetf = 0;\n"
         "  tf.user_data = NULL;\n"
         "  vpi_register_systf(&tf);\n"
         "}\n" << std::endl;

  out << "void (*vlog_startup_routines[])() = {\n"
         "  register_verilog_functions,\n"
         "  0\n"
         "};";
}

void tracerimpl::toVPI_v(std::ofstream& out, const std::string& moduleFileName) const {
  //--
  auto netlist_name = [&](lnodeimpl* node)->std::string {
    std::stringstream ss;
    verilogwriter::print_node_name(ss, node);
    return ss.str();
  };

  //--
  auto get_tap_path = [&](tapimpl* node) {    
    if (is_single_context_) {
      auto sname = netlist_name(node);
      return stringf("%s_%d.%s", node->ctx()->name().c_str(), node->ctx()->id(), sname.c_str());
    } else {
      auto pos  = node->name().find_last_of('/');
      auto path = node->name().substr(0, pos);
      std::replace(path.begin(), path.end(), '/', '.');
      auto sname = identifier_from_string(node->name().substr(pos+1));
      return stringf("%s.%s", path.c_str(), sname.c_str());;
    }
  };

  //--
  auto get_signal_name = [&](ioportimpl* node) {
    auto path = node->name();
    if (!is_single_context_) {
      auto sname = remove_path(path);
      if (sname == "clk" || sname == "reset")
        return sname;
      if (1 == contexts_.size()) {
        path = sname;
      }
    }
    path = identifier_from_string(path);
    return path;
  };

  //--
  auto find_signal_name = [&](ioportimpl* node) {
    if ((node->name() == "clk") || (node->name() == "reset"))
      return node->name();
    auto name = node->name();
    if (!is_single_context_) {
      name = stringf("%s_%d/%s", node->ctx()->name().c_str(), node->ctx()->id(), name.c_str());
    }
    for (auto signal : signals_) {      
      if (signal->name() == name)
        return get_signal_name(signal);
    }
    std::abort();
    return std::string();
  };

  //--
  auto print_type = [](std::ostream& out, ioimpl* node) {
    out << (type_input == node->type() ? "reg" : "wire");
    if (node->size() > 1)
      out << "[" << (node->size() - 1) << ":0]";
  };

  //--
  auto print_module = [&](std::ostream& out, context* ctx) {
    auto_separator sep(", ");
    out << ctx->name() << " " << ctx->name() << "_" << ctx->id() << "(";
    for (auto node : ctx->inputs()) {
      auto input = reinterpret_cast<inputimpl*>(node);
      out << sep << '.' << identifier_from_string(input->name()) << "(" << find_signal_name(input) << ")";
    }
    for (auto node : ctx->outputs()) {
      auto output = reinterpret_cast<outputimpl*>(node);
      out << sep << '.' << identifier_from_string(output->name()) << "(" << find_signal_name(output) << ")";
    }
    out << ");" << std::endl;
    return true;
  };

  // log header
  out << "`timescale 1ns/1ns" << std::endl;
  out << "`include \"" << moduleFileName << "\"" << std::endl << std::endl;
  out << "module testbench();" << std::endl << std::endl;

  {
    auto_indent indent(out);
    int has_clock = 0;
    int has_taps = 0;

    // declare signals
    for (auto signal : signals_) {
      print_type(out, signal);
      auto name = get_signal_name(signal);
      out << " " << name << ";" << std::endl;
      has_clock |= (name == "clk");
      has_taps |= (type_tap == signal->type());
    }
    out << std::endl;

    // declare modules
    for (auto ctx : contexts_) {
      print_module(out, ctx);
      out << std::endl;
    }

    if (has_taps) {
      for (auto signal : signals_) {
        if (type_tap != signal->type())
          continue;
        out << "assign " << get_signal_name(signal) << " = "
            << get_tap_path(reinterpret_cast<tapimpl*>(signal)) << ";" << std::endl;
      }
      out << std::endl;
    }

    // declare clock process
    if (has_clock) {
      out << "always begin" << std::endl;
      {
        auto_indent indent1(out);
        out << "#1 clk = !clk;" << std::endl;
      }
      out << "end" << std::endl << std::endl;
    }

    // declare simulation process
    out << "initial begin" << std::endl;
    out << "$eval_callback;" << std::endl;
    if (has_clock) {
      out << "clk = 0;" << std::endl;
    }
    out << "end" << std::endl << std::endl;
  }

  // log footer
  out << "endmodule" << std::endl;
}

void tracerimpl::toVPI(const std::string& vfile, 
                       const std::string& cfile, 
                       const std::string& moduleFileName) const {
  {
    std::ofstream out(vfile);
    this->toVPI_v(out, moduleFileName);
  }
  {
    std::ofstream out(cfile);
    this->toVPI_c(out);
  }   
}

///////////////////////////////////////////////////////////////////////////////

ch_tracer::ch_tracer(const std::vector<device_base>& devices)
  : ch_simulator(new tracerimpl(devices)) {
  impl_->initialize();
}

ch_tracer::ch_tracer(simulatorimpl* impl) : ch_simulator(impl) {}

ch_tracer::ch_tracer(const ch_tracer& other) : ch_simulator(other) {}

ch_tracer::ch_tracer(ch_tracer&& other) : ch_simulator(std::move(other)) {}

ch_tracer::~ch_tracer() {}

ch_tracer& ch_tracer::operator=(const ch_tracer& other) {
  ch_simulator::operator=(other);
  return *this;
}

ch_tracer& ch_tracer::operator=(ch_tracer&& other) {
  ch_simulator::operator=(std::move(other));
  return *this;
}

void ch_tracer::toText(std::ofstream& out) {
  return reinterpret_cast<tracerimpl*>(impl_)->toText(out);
}

void ch_tracer::toVCD(std::ofstream& out) {
  return reinterpret_cast<tracerimpl*>(impl_)->toVCD(out);
}

void ch_tracer::toVerilog(std::ofstream& out,
                          const std::string& moduleFileName,
                          bool passthru) {
  return reinterpret_cast<tracerimpl*>(impl_)->toVerilog(out, moduleFileName, passthru);
}

void ch_tracer::toVerilator(std::ofstream& out,
                            const std::string& moduleTypeName) {
  return reinterpret_cast<tracerimpl*>(impl_)->toVerilator(out, moduleTypeName);
}

void ch_tracer::toSystemC(std::ofstream& out,
                          const std::string& moduleTypeName) {
  return reinterpret_cast<tracerimpl*>(impl_)->toSystemC(out, moduleTypeName);
}

void ch_tracer::toVPI(const std::string& vfile, 
                      const std::string& cfile, 
                      const std::string& moduleFileName) {
  return reinterpret_cast<tracerimpl*>(impl_)->toVPI(vfile, cfile, moduleFileName);
}