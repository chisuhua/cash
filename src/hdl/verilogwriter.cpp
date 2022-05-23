﻿#include "verilogwriter.h"
#include "codegen.h"
#include "context.h"
#include "compile.h"
#include "deviceimpl.h"
#include "moduleimpl.h"
#include "litimpl.h"
#include "cdimpl.h"
#include "regimpl.h"
#include "memimpl.h"
#include "ioimpl.h"
#include "selectimpl.h"
#include "proxyimpl.h"
#include "memimpl.h"
#include "opimpl.h"
#include "assertimpl.h"
#include "udfimpl.h"
#include "udf.h"

using namespace ch::internal;

static bool IsRegType(lnodeimpl* node) {
  auto type = node->type();
  return (type_reg == type && !reinterpret_cast<regimpl*>(node)->is_pipe())
      || (type_msrport == type)
      || (type_mem == type)
      || (type_sel == type && !reinterpret_cast<selectimpl*>(node)->is_ternary());
}

static bool IsUseBeforeDef(lnodeimpl* node, const lnodeimpl* use) {
  return (use->id() < node->id());
}

static bool is_inline_literal(lnodeimpl* node) {
  assert(type_lit == node->type());
  return (node->size() <= 64);
}

static auto DataWidth(lnodeimpl* node) {
  auto type = node->type();
  return (type_mem == type) ? reinterpret_cast<memimpl*>(node)->data_width() : node->size();
}

///////////////////////////////////////////////////////////////////////////////

verilogwriter::verilogwriter(context* ctx) : ctx_(ctx) {
  for (auto node : ctx->nodes()) {
    for (auto& src : node->srcs()) {
      uses_[src.id()].insert(node);
    }
  }
}

verilogwriter::~verilogwriter() {}

bool verilogwriter::is_inline_subscript(lnodeimpl* node) const {
  assert(type_proxy == node->type());
  if (reinterpret_cast<proxyimpl*>(node)->ranges().size() > 1)
    return false;
  if (type_proxy == node->src(0).impl()->type())
    return false;
  auto it = uses_.find(node->id());
  if (it != uses_.end()) {
    for (lnodeimpl* use : it->second) {
      if (type_proxy == use->type()
       || type_marport == use->type()
       || type_tap == use->type()
       || (type_op == use->type()       
        && ch_op::pad == reinterpret_cast<opimpl*>(use)->op()))
        return false;
    }
  }
  return true;
}

void verilogwriter::print(std::ostream& out,
                          std::unordered_set<std::string_view>& visited) {  
  // print dependent modules
  for (auto node : ctx_->modules()) {
    auto module = reinterpret_cast<moduleimpl*>(node);
    auto target = module->target();
    // ensure single instantiation
    if (visited.count(target->name()))
      continue;
    visited.insert(target->name());
    verilogwriter child_module(target);
    child_module.print(out, visited);
    out << std::endl;
  }

  // print header
  this->print_header(out);

  // print body
  this->print_body(out);

  // print footer
  this->print_footer(out);
}

void verilogwriter::print_header(std::ostream& out) {
  //
  // ports declaration
  //
  out << "module " << ctx_->name() << '(';
  {
    auto_indent indent(out);
    auto_separator sep(",");

    auto clk = ctx_->sys_clk();
    if (clk) {
      out << sep << std::endl;
      this->print_port(out, clk);
    }

    auto reset = ctx_->sys_reset();
    if (reset) {
      out << sep << std::endl;
      this->print_port(out, reset);
    }

    // gather all ports and sort them in user-defined order
    std::vector<lnodeimpl*> ports;
    ports.reserve(ctx_->inputs().size() + ctx_->outputs().size());
    ports.insert(ports.end(), ctx_->inputs().begin(), ctx_->inputs().end());
    ports.insert(ports.end(), ctx_->outputs().begin(), ctx_->outputs().end());
    std::sort(ports.begin(),
              ports.end(),
              [](lnodeimpl* a, lnodeimpl*b) {return a->id() < b->id();}
      );
    for (auto node : ports) {
      auto port = reinterpret_cast<ioimpl*>(node);
      if (port == clk || port == reset)
        continue;
      out << sep << std::endl;
      this->print_port(out, port);
    }
  }
  out << std::endl << ");" << std::endl;
}

void verilogwriter::print_port(std::ostream& out, ioimpl* node) {
  auto type = node->type();
  switch (type) {
  case type_input:
    out << "input";
    break;
  case type_output:
    out << "output";
    break;
  default:
    assert(false);
  }

  out << " ";
  this->print_type(out, node);

  out << " ";
  this->print_name(out, node);
}

void verilogwriter::print_body(std::ostream& out) {
  //
  // declaration
  //
  {
    auto_indent indent(out);

    std::unordered_set<uint32_t> visited;
    int written = 0;
    for (auto node : ctx_->literals()) {
      written |= this->print_decl(out, node, visited);
    }
    for (auto node : ctx_->nodes()) {
      written |= this->print_decl(out, node, visited);
    }
    if (written) {
      out << std::endl;
    }
  }

  //
  // UDF declarations
  //
  {
    auto_indent indent(out);
    int written = 0;
    for (auto node : ctx_->udfs()) {
      written |= this->print_udf(out, reinterpret_cast<udfimpl*>(node), udf_verilog::declaration);
    }
    if (written) {
      out << std::endl;
    }
  }

  //
  // body logic
  //
  {
    auto_indent indent(out);
    int written = 0;
    for (auto node : ctx_->nodes()) {
      written |= this->print_logic(out, node);
    }
    if (written) {
      out << std::endl;
    }
  }
}

void verilogwriter::print_footer(std::ostream& out) {
  //
  // output assignment
  //
  {
    auto_indent indent(out);
    bool written = false;

    for (auto output : ctx_->outputs()) {
      out << "assign ";
      this->print_name(out, output);
      out << " = ";
      this->print_name(out, reinterpret_cast<outputimpl*>(output)->src(0).impl());
      out << ";" << std::endl;
      written = true;
    }

    if (written) {
      out << std::endl;
    }
  }

  out << "endmodule" << std::endl;
}

bool verilogwriter::print_decl(std::ostream& out,
                               lnodeimpl* node,
                               std::unordered_set<uint32_t>& visited,
                               lnodeimpl* inline_node) {
  if (visited.count(node->id()))
    return false;

  auto type = node->type();
  switch (type) {
  case type_lit:
    visited.insert(node->id());
    if (is_inline_literal(node))
      return false;
    this->print_type(out, node);
    out << " ";
    this->print_name(out, node);
    out << " = ";
    this->print_value(out, reinterpret_cast<litimpl*>(node)->value(), true);
    out << ";" << std::endl;
    return true;
  case type_proxy:
    if (this->is_inline_subscript(node)) {
      visited.insert(node->id());
      return false;
    }
    [[fallthrough]];
  case type_modpin:
  case type_modpout:
  case type_op:
  case type_sel:
  case type_reg:
  case type_msrport:
  case type_marport:  
  case type_mem:    
  case type_tap:
  case type_bypass:  
  case type_udfout:
    if (type_modpin == type) {
      // inline mode if already defined at use location
      auto modpin = reinterpret_cast<moduleportimpl*>(node);
      auto src  =modpin->src(0).impl();
      if (!IsUseBeforeDef(src, modpin))
        return false;
    }    

    if (inline_node != nullptr
     && (IsRegType(inline_node) != IsRegType(node)
      || DataWidth(inline_node) != DataWidth(node))) {
      return false;    
    }

    if (inline_node != nullptr) {
      out << ", ";
    } else {
      this->print_type(out, node);
      out << " ";
    }

    this->print_name(out, node);

    if (type_mem == type) {
      auto mem = reinterpret_cast<memimpl*>(node);
      out << " [0:" << (mem->num_items() - 1) << "]";
      if (mem->is_logic_rom()
       && mem->force_logic_ram()) {
        out << " /* synthesis";
        if (mem->size() >= 20*1024) {
          out << " ramstyle = \"M20K\"";
        } else {
          out << " ramstyle = \"MLAB\"";
        }
        if (mem->has_init_data()) {
          const auto& value = mem->init_data();
          auto data_width = mem->data_width();
          auto num_items = mem->num_items();

          std::stringstream ss;
          this->print_name(ss, mem);
          ss << ".mif";
          auto filename = ss.str();

          out << ", ram_init_file = \"" << filename  << "\"";

          std::ofstream out_mif(filename);
          out_mif << "WIDTH = " << data_width << ";" << std::endl;
          out_mif << "DEPTH = " << num_items << ";" << std::endl;
          out_mif << "ADDRESS_RADIX = HEX;" << std::endl;
          out_mif << "DATA_RADIX = HEX;" << std::endl;
          out_mif << "CONTENT BEGIN" << std::endl;
          out_mif << std::hex;
          for (uint32_t i = 0; i < num_items; ++i) {
            out_mif << i << " : ";
            this->print_value(out_mif, value, true, i * data_width, data_width, true);
            out_mif << ";" << std::endl;
          }
          out_mif << "END;"<< std::endl;
        }
        out << " */";
      }
    }

    visited.insert(node->id());
    
    if (inline_node == nullptr) {      
      if (platform::self().cflags() & ch_flags::codegen_sloc) {
        out << ";";
        auto& sloc = node->sloc();
        if (!sloc.empty()) {
          out << " // " << sloc;
        }
      } else {        
        for (auto other : node->ctx()->nodes()) {
          this->print_decl(out, other, visited, node);
        }       
        out << ";";
      }
      out << std::endl;
    }
    return true;
    
  case type_module:
  case type_input:
  case type_output:
  case type_cd:
  case type_mwport:
  case type_udfc:
  case type_udfs:
  case type_udfin:
  case type_assert:
  case type_print:
  case type_time:
    visited.insert(node->id());
    break;
  default:
    assert(false);
  }  
  return false;
}

bool verilogwriter::print_logic(std::ostream& out, lnodeimpl* node) {
  auto type = node->type();
  switch (type) {
  case type_proxy:
    return this->print_proxy(out, reinterpret_cast<proxyimpl*>(node));
  case type_op:
    return this->print_op(out, reinterpret_cast<opimpl*>(node));
  case type_sel:
    return this->print_select(out, reinterpret_cast<selectimpl*>(node));
  case type_reg:
    return this->print_reg(out, reinterpret_cast<regimpl*>(node));
  case type_mem:
    return this->print_mem(out, reinterpret_cast<memimpl*>(node));
  case type_module:
    return this->print_module(out, reinterpret_cast<moduleimpl*>(node));
  case type_modpin:
  case type_modpout:
    return this->print_modport(out, reinterpret_cast<moduleportimpl*>(node));
  case type_udfc:
  case type_udfs:
    return this->print_udf(out, reinterpret_cast<udfimpl*>(node), udf_verilog::body);
  case type_tap:
    return this->print_tap(out, reinterpret_cast<tapimpl*>(node));
  case type_bypass:
    return this->print_bypass(out, reinterpret_cast<bypassimpl*>(node));
  default:
    assert(false);
  case type_lit:
  case type_input:
  case type_output:
  case type_cd:
  case type_msrport:
  case type_marport:
  case type_mwport:  
  case type_udfin:
  case type_udfout:
  case type_assert:
  case type_print:
  case type_time:
    return false;
  }  
}

bool verilogwriter::print_proxy(std::ostream& out, proxyimpl* node) {
  if (this->is_inline_subscript(node))
    return false;
  out << "assign ";
  this->print_name(out, node);
  out << " = ";
  this->print_proxy_value(out, node);
  out << ";" << std::endl;
  return true;
}

void verilogwriter::print_proxy_value(std::ostream& out, proxyimpl* node) {
  const auto& ranges = node->ranges();
  uint32_t dst_offset = node->size();

  //--
  auto print_range = [&](const proxyimpl::range_t& range) {
    dst_offset -= range.length;
    assert(range.dst_offset == dst_offset);
    auto& src = node->src(range.src_idx);
    if (src.impl()->type() == type_lit) {
      sdata_type value(range.length);
      auto src_data = reinterpret_cast<litimpl*>(src.impl())->value().words();
      bv_copy(value.words(), 0, src_data, range.src_offset, range.length);
      print_value(out, value, true);
    } else {
      this->print_name(out, src.impl());
      if (range.length < src.size()) {
        out << "[";
        if (range.length > 1) {
          out << (range.src_offset + range.length - 1) << ":" << range.src_offset;
        } else {
          out << range.src_offset;
        }
        out << "]";
      };
    }
  };

  if (ranges.size() > 1) {
    out << "{";
    auto_separator sep(", ");
    for (int i = ranges.size() - 1; i >= 0; --i) {
      out << sep;
      print_range(ranges[i]);
    }
    out << "}";
  } else {
    print_range(ranges[0]);
  }
}

bool verilogwriter::print_op(std::ostream& out, opimpl* node) {
  //--
  auto print_signed_operand = [&](unsigned index) {
    if (node->is_signed())
      out << "$signed(";
    this->print_name(out, node->src(index).impl());
    if (node->is_signed())
      out << ")";
  };

  auto op = node->op();
  if (op == ch_op::pad) {
    if (node->is_signed())
      this->print_sext(out, node);
    else
      this->print_zext(out, node);
  } else {
    out << "assign ";
    this->print_name(out, node);
    out << " = ";
    if (CH_OP_ARY(op) == op_flags::binary) {
      if (CH_OP_CLASS(op) == op_flags::shift
       || CH_OP_CLASS(op) == op_flags::relational
       || CH_OP_CLASS(op) == op_flags::logical
       || CH_OP_CLASS(op) == op_flags::arithmetic) {
        print_signed_operand(0);
        out << " ";
        this->print_operator(out, op);
        out << " ";
        if (op == ch_op::shl || op == ch_op::shr) {
          this->print_name(out, node->src(1).impl());
        } else {
          print_signed_operand(1);
        }
      } else {
        this->print_name(out, node->src(0).impl());
        out << " ";
        this->print_operator(out, op);
        out << " ";
        this->print_name(out, node->src(1).impl());
      }
      out << ";" << std::endl;
    } else {
      assert(CH_OP_ARY(op) == op_flags::unary);
      this->print_operator(out, op);
      this->print_name(out, node->src(0).impl());
      out << ";" << std::endl;
    }
  }
  return true;
}

bool verilogwriter::print_zext(std::ostream& out, opimpl* node) {
  out << "assign ";
  this->print_name(out, node);
  out << " = {{" << node->size() - node->src(0).size() << "{1'b0}}, ";
  this->print_name(out, node->src(0).impl());
  out << "};" << std::endl;
  return true;
}

bool verilogwriter::print_sext(std::ostream& out, opimpl* node) {
  out << "assign ";
  this->print_name(out, node);
  out << " = {{" << node->size() - node->src(0).size() << "{";
  this->print_name(out, node->src(0).impl());
  out << "[" << (node->src(0).size() - 1) << "]}}, ";
  this->print_name(out, node->src(0).impl());
  out << "};" << std::endl;
  return true;
}

bool verilogwriter::print_select(std::ostream& out, selectimpl* node) {
  bool has_key = node->has_key();
  uint32_t i = has_key ? 1 : 0;  
  if (node->is_ternary()) {
    out << "assign ";
    this->print_name(out, node);
    out << " = ";
    if (has_key) {
      out << "(";
      this->print_name(out, node->src(0).impl());
      out << " == ";
      this->print_name(out, node->src(1).impl());
      out << ")";
    } else {
      this->print_name(out, node->src(0).impl());
    }
    out << " ? ";
    this->print_name(out, node->src(i+1).impl());
    out << " : ";
    this->print_name(out, node->src(i+2).impl());
    out << ";" << std::endl;
  } else {
    uint32_t last = node->num_srcs() - 1;
    out << "always @(*) begin" << std::endl;
    if (has_key) {
      auto_indent indent1(out);
      out << "case (";
      this->print_name(out, node->src(0).impl());
      out << ")" << std::endl;
      {
        auto_indent indent2(out);
        for (; i < last; i += 2) {
          assert(type_lit == node->src(i).impl()->type());
          auto& value = reinterpret_cast<litimpl*>(node->src(i).impl())->value();
          print_value(out, value, false);
          out << ": ";
          this->print_name(out, node);
          out << " = ";
          this->print_name(out, node->src(i + 1).impl());
          out << ";" << std::endl;
        }
        out << "default: ";
        this->print_name(out, node);
        out << " = ";
        this->print_name(out, node->src(i).impl());
        out << ";" << std::endl;
      }
      out << "endcase" << std::endl;
    } else {
      auto_indent indent1(out);
      for (; i < last; i += 2) {
        out << (0 == i ? "if (" : "else if (");
        this->print_name(out, node->src(i).impl());
        out << ")" << std::endl;
        {
          auto_indent indent2(out);
          this->print_name(out, node);
          out << " = ";
          this->print_name(out, node->src(i + 1).impl());
          out << ";" << std::endl;
        }
      }
      out << "else" << std::endl;
      {
        auto_indent indent2(out);
        this->print_name(out, node);
        out << " = ";
        this->print_name(out, node->src(i).impl());
        out << ";" << std::endl;
      }
    }
    out << "end" << std::endl;
  }
  return true;
}

bool verilogwriter::print_reg(std::ostream& out, regimpl* node) {
  //--
  auto print_sr_name = [&]() {
    this->print_name(out, node);
    out << "_sr";
  };

  if (node->is_pipe()) {
    out << "reg";
    if (node->size() > 1) {
      out << "[" << (node->size() - 1) << ":0]";
    }
    out << " ";
    print_sr_name();
    out << " [" << (node->length()-1) << ":0];" << std::endl;
    out << "always @ (";
    auto cd = reinterpret_cast<cdimpl*>(node->cd().impl());
    this->print_cdomain(out, cd);
    out << ") begin" << std::endl;
    {
      auto_indent indent(out);

      if (node->has_init_data()) {
        out << "if (";
        this->print_name(out, node->reset().impl());
        out << ") begin" << std::endl;        
        indent.push();
        for (uint32_t i = 0, n = node->length(); i < n; ++i) {
          print_sr_name();
          out << "[" << i << "] <= ";
          this->print_name(out, node->init_data().impl());
          out << ";" << std::endl;
        }
        indent.pop();
        out << "end" << std::endl;
        out << "else" << std::endl;
      }

      if (node->has_enable()) {
        out << "if (";
        this->print_name(out, node->enable().impl());
        out << ") begin" << std::endl;
        indent.push();
      } else {
        if (node->has_init_data()) {
          out << "begin" << std::endl;
          indent.push();
        }
      }

      for (uint32_t i = 0, n = node->length() - 1; i < n; ++i) {
        print_sr_name();
        out << "[" << (node->length() - i - 1) << "] <= ";
        print_sr_name();
        out << "[" << (node->length() - i - 2) << "];" << std::endl;
      }

      print_sr_name();
      out << "[0] <= ";
      this->print_name(out, node->next().impl());
      out << ";" << std::endl;
      if (node->has_init_data() || node->has_enable()) {
        indent.pop();
        out << "end" << std::endl;
      }
    }
    out << "end" << std::endl;
    out << "assign ";
    this->print_name(out, node);
    out << " = ";
    print_sr_name();
    out << "[" << (node->length() - 1) << "];" << std::endl;
  } else {
    out << "always @ (";
    auto cd = reinterpret_cast<cdimpl*>(node->cd().impl());
    this->print_cdomain(out, cd);
    out << ") begin" << std::endl;
    {
      auto_indent indent(out);

      if (node->has_init_data()) {
        out << "if (";
        this->print_name(out, node->reset().impl());
        out << ")" << std::endl;
        indent.push();
        this->print_name(out, node);
        out << " <= ";
        this->print_name(out, node->init_data().impl());
        out << ";" << std::endl;
        indent.pop();
        out << "else" << std::endl;
      }

      if (node->has_enable()) {
        out << "if (";
        this->print_name(out, node->enable().impl());
        out << ")" << std::endl;
        indent.push();
      } else {
        if (node->has_init_data()) {
          indent.push();
        }
      }

      this->print_name(out, node);
      out << " <= ";
      this->print_name(out, node->next().impl());
      out << ";" << std::endl;
      if (node->has_init_data() || node->has_enable()) {
        indent.pop();
      }
    }
    out << "end" << std::endl;
  }
  return true;
}

bool verilogwriter::print_module(std::ostream& out, moduleimpl* node) {
  //--
  auto find_modpin = [&](inputimpl* port)->moduleportimpl* {
    for (auto& input : node->inputs()) {
      auto b = reinterpret_cast<moduleportimpl*>(input.impl());
      if (b->ioport().impl() == port)
        return b;
    }
    return nullptr;
  };

  //--
  auto find_modpout = [&](outputimpl* port)->moduleportimpl* {
    for (auto& output : node->outputs()) {
      auto b = reinterpret_cast<moduleportimpl*>(output.impl());
      if (b->ioport().impl() == port)
        return b;
    }
    return nullptr;
  };

  auto_separator sep(", ");
  auto m = node->target();
  out << m->name() << " ";
  print_name(out, node);
  out << "(";
  {
    auto_indent indent(out);
    auto_separator sep2(",");
    for (auto n : m->inputs()) {
      auto input = reinterpret_cast<inputimpl*>(n);
      out << sep2 << std::endl << '.' << identifier_from_string(input->name()) << "(";
      auto modpin = find_modpin(input);
      if (modpin) {
        auto src = modpin->src(0).impl();
        if (IsUseBeforeDef(src, modpin)) {
          this->print_name(out, modpin);
        } else {
          this->print_name(out, src);          
        }
      }
      out << ")";
    }
    for (auto n : m->outputs()) {
      auto output = reinterpret_cast<outputimpl*>(n);
      out << sep2 << std::endl << '.' << identifier_from_string(output->name()) << "(";
      auto modport = find_modpout(output);
      if (modport) {
        this->print_name(out, modport);
      }
      out << ")";
    }
  }
  out << ");" << std::endl;
  return true;
}

bool verilogwriter::print_modport(std::ostream& out, moduleportimpl* node) {
  // outputs are sourced via binding already
  if (node->type() == type_modpout)
    return false;

   auto src = node->src(0).impl();
   if (!IsUseBeforeDef(src, node))
    return false;

  out << "assign ";
  this->print_name(out, node);
  out << " = ";
  this->print_name(out, src);
  out << ";" << std::endl;
  std::flush(out);
  return true;
}

bool verilogwriter::print_cdomain(std::ostream& out, cdimpl* cd) {
  out << (cd->pos_edge() ? "posedge" : "negedge") << " ";
  this->print_name(out, cd->clk().impl());
  return true;
}

bool verilogwriter::print_mem(std::ostream& out, memimpl* node) {
  //--
  auto print_port = [&](std::ostream& out, memportimpl* p) {
    this->print_name(out, p->mem());
    out << "[";
    this->print_name(out, p->addr().impl());
    out << "]";
  };

  //
  // initialization data
  //
  if (node->has_init_data()
   && !node->force_logic_ram()) {
    out << "initial begin" << std::endl;
    {
      auto_indent indent(out);
      const auto& value = node->init_data();
      auto data_width = node->data_width();
      auto num_items = node->num_items();
      for (uint32_t i = 0; i < num_items; ++i) {
        this->print_name(out, node);
        out << "[" << i << "] = ";
        this->print_value(out, value, true, i * data_width, data_width);
        out << ";" << std::endl;
      }
    }
    out << "end" << std::endl;
  }

  //
  //  ports logic
  //
  for (auto p : node->rdports()) {
    if (type_msrport == node->type()) {
      out << "always @ (";
      this->print_cdomain(out, reinterpret_cast<cdimpl*>(p->cd().impl()));
      out << ") begin" << std::endl;
      {
        auto_indent indent1(out);
        if (p->has_enable()) {
          out << "if (";
          this->print_name(out, p->enable().impl());
          out << ") begin" << std::endl;
          {
            auto_indent indent2(out);
            this->print_name(out, p);
            out << " <= ";
            print_port(out, p);
            out << ";" << std::endl;
          }
          out << "end" << std::endl;
        } else {
          this->print_name(out, p);
          out << " <= ";
          print_port(out, p);
          out << ";" << std::endl;
        }
      }
      out << "end" << std::endl;
    } else {
      out << "assign ";
      this->print_name(out, p);
      out << " = ";
      print_port(out, p);
      out << ";" << std::endl;
    }
  }

  for (auto p : node->wrports()) {
    out << "always @ (";
    this->print_cdomain(out, reinterpret_cast<cdimpl*>(p->cd().impl()));
    out << ") begin" << std::endl;
    {
      auto_indent indent1(out);
      if (p->has_enable()) {
        out << "if (";
        this->print_name(out, p->enable().impl());
        out << ") begin" << std::endl;
        {
          auto_indent indent2(out);
          print_port(out, p);
          out << " <= ";
          this->print_name(out, p->wdata().impl());
          out << ";" << std::endl;
        }
        out << "end" << std::endl;
      } else {
        print_port(out, p);
        out << " <= ";
        this->print_name(out, p->wdata().impl());
        out << ";" << std::endl;
      }
    }
    out << "end" << std::endl;
  }
  return true;
}

bool verilogwriter::print_tap(std::ostream& out, tapimpl* node) {
  out << "assign ";
  this->print_name(out, node);
  out << " = ";
  this->print_name(out, node->target().impl());
  out << ";" << std::endl;
  return true;
}

bool verilogwriter::print_bypass(std::ostream& out, bypassimpl* node) {
  //--
  std::function<std::string (lnodeimpl*, context*)> full_path =
      [&](lnodeimpl* bypass, context* ctx)->std::string {
    if (bypass->ctx() == ctx) {
      for (auto ex : ctx->nodes()) {
        if (ex->id() == bypass->id()) {
          std::stringstream ss;
          print_name(ss, bypass);
          return ss.str();
        }
      }
    }

    for (auto module : ctx->modules()) {
      auto ret = full_path(bypass, reinterpret_cast<moduleimpl*>(module)->target());
      if (!ret.empty()) {
        std::stringstream ss;
        print_name(ss, module);
        ss << '.' << ret;
        return ss.str();
      }
    }

    return "";
  };

  out << "assign ";
  this->print_name(out, node);
  auto path = full_path(node->target(), node->ctx());
  out << " = " << path << ";" << std::endl;
  return true;
}

bool verilogwriter::print_udf(std::ostream& out, udfimpl* node, udf_verilog mode) {
  std::unordered_map<std::string, std::string> dic;

  //--
  auto dict_add = [&](const std::string& key, lnodeimpl* value) {
    std::ostringstream os;
    this->print_name(os, value);
    dic[key] = os.str();
  };

  dic["id"] = stringf("%d", node->id());

  for (auto& input : node->inputs()) {
    dict_add(input.impl()->name(), input.impl()->src(0).impl());
  }

  for (auto& output : node->outputs()) {
    dict_add(output.impl()->name(), output.impl());
  }

  if (type_udfs == node->type()) {
    auto udfs = reinterpret_cast<udfsimpl*>(node);
    auto cd = reinterpret_cast<cdimpl*>(udfs->cd().impl());
    dict_add("clk", cd->clk().impl());
    dict_add("reset", udfs->reset().impl());
  }

  std::string code;
  bool changed;
  {
    // load code template
    std::ostringstream os;
    udf_vostream udf_os(os.rdbuf());
    changed = node->udf()->to_verilog(udf_os, mode);
    code = os.str();
  }

  if (changed) {
    // replace tokens
    static const std::string Identifier("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ01234567890_.");
    std::string::size_type start = 0, pos = 0;
    auto len = code.length();
    while ((pos = code.find('$', pos)) != std::string::npos) {
      auto k = pos + 1;
      if (k < len) {
        auto end = code.find_first_not_of(Identifier, k);
        if (end == std::string::npos || end > k) {
          auto key = code.substr(k, end - k);
          auto it = dic.find(key);
          if (it != dic.end()) {
            out << code.substr(start, pos - start);
            out << it->second;
            start = end;
            pos = end;
            if (pos == std::string::npos)
              break;
            continue;
          }
        }
      }

      out << code.substr(start, k - start);
      start = k;
      pos = k;
    }
    if (start != std::string::npos) {
      out << code.substr(start);
    }
    out << std::endl;
  }

  return changed;
}

void verilogwriter::print_node_name(std::ostream& out, lnodeimpl* node) {
  switch (node->type()) {  
  case type_time:
    out << "$time";
    break;
  default:
    node->unique_name(out);
    break;
  }
}

void verilogwriter::print_name(std::ostream& out, lnodeimpl* node, bool noinline) {
  auto type = node->type();
  switch (type) {
  case type_proxy:    
    if (!noinline && this->is_inline_subscript(node)) {
      this->print_proxy_value(out, reinterpret_cast<proxyimpl*>(node));
    } else {
      print_node_name(out, node);
    }
    break;
  case type_lit:
    if (!noinline && is_inline_literal(node)) {
      auto& value = reinterpret_cast<litimpl*>(node)->value();
      print_value(out, value, true);
    } else {
      print_node_name(out, node);
    }
    break;
  default:
    print_node_name(out, node);
    break;
  }
}

void verilogwriter::print_type(std::ostream& out, lnodeimpl* node) {  
  out << (IsRegType(node) ? "reg" : "wire");
  auto type = node->type();
  if (type == type_mem) {
    auto mem = reinterpret_cast<memimpl*>(node);
    auto data_width = mem->data_width();
    if (data_width > 1) {
      out << "[" << (data_width - 1) << ":0]";
    }
  } else {
    if (node->size() > 1) {
      out << "[" << (node->size() - 1) << ":0]";
    }
  }
}

void verilogwriter::print_value(std::ostream& out,
                                const sdata_type& value,
                                bool skip_zeros,
                                uint32_t offset,
                                uint32_t size,
                                bool digit_only) {
  if (0 == size) {
    size = value.size();
  }

  if (!digit_only) {
    out << size << "'h";
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
}

void verilogwriter::print_operator(std::ostream& out, ch_op op) {
  switch (op) {
  case ch_op::eq:   out << "=="; break;
  case ch_op::ne:   out << "!="; break;

  case ch_op::lt:   out << "<"; break;
  case ch_op::gt:   out << ">"; break;
  case ch_op::le:   out << "<="; break;
  case ch_op::ge:   out << ">="; break;

  case ch_op::notl: out << "!"; break;
  case ch_op::andl: out << "&&"; break;
  case ch_op::orl:  out << "||"; break;

  case ch_op::inv:  out << "~"; break;
  case ch_op::andb: out << "&"; break;
  case ch_op::orb:  out << "|"; break;
  case ch_op::xorb: out << "^"; break;

  case ch_op::andr: out << "&"; break;
  case ch_op::orr:  out << "|"; break;
  case ch_op::xorr: out << "^"; break;

  case ch_op::shl:  out << "<<"; break;
  case ch_op::shr:  out << ">>"; break;

  case ch_op::neg:  out << "-"; break;
  case ch_op::add:  out << "+"; break;
  case ch_op::sub:  out << "-"; break;
  case ch_op::mul: out << "*"; break;
  case ch_op::div:  out << "/"; break;
  case ch_op::mod:  out << "%"; break;

  default:
    assert(false);
  }
}

///////////////////////////////////////////////////////////////////////////////

std::function<bool (udf_vostream& out, context*, std::unordered_set<uint32_t>&)>
  //--
  print_udf_dependencies = [](udf_vostream& out,
                              context* ctx,
                              std::unordered_set<uint32_t>& visited) {
  bool changed = false;
  for (auto node : ctx->modules()) {
    auto module = reinterpret_cast<moduleimpl*>(node);
    if (print_udf_dependencies(out, module->target(), visited))
      changed = true;
  }


  for (auto node : ctx->udfs()) {
    auto udfs = reinterpret_cast<udfsimpl*>(node);
    if (0 == visited.count(udfs->id())) {
      if (udfs->udf()->to_verilog(out, udf_verilog::header)) {
        out << std::endl;
        changed =true;
      }
      visited.insert(udfs->id());
    }
  }

  return changed;
};

void ch::internal::ch_toVerilog(std::ostream& out, const device_base& device) {
  //--
  std::unordered_set<std::string_view> visited;
  std::unordered_set<uint32_t> udf_visited;

  auto ctx = device.impl()->ctx();
  if (ctx->modules().size() 
   && (platform::self().cflags() & ch_flags::codegen_merged) != 0) {
    auto merged_ctx = new context(ctx->name());
    compiler compiler(merged_ctx);
    compiler.create_merged_context(ctx);
    compiler.optimize();
    ctx = merged_ctx;
  }      
  ctx->acquire();
  
  visited.insert(ctx->name());
  udf_vostream udf_os(out.rdbuf());
  if (print_udf_dependencies(udf_os, ctx, udf_visited)) {
    out << std::endl;
  }

  verilogwriter writer(ctx);  
  writer.print(out, visited);

  ctx->release();
}
