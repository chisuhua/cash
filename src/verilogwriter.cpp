#include "verilogwriter.h"
#include "context.h"
#include "litimpl.h"
#include "regimpl.h"
#include "memimpl.h"
#include "ioimpl.h"
#include "selectimpl.h"
#include "proxyimpl.h"
#include "memimpl.h"
#include "aluimpl.h"
#include "assertimpl.h"
#include "tickimpl.h"
#include <cstring>

using namespace cash::internal;

verilogwriter::verilogwriter(std::ostream& out) : out_(out) {
  //--
}

verilogwriter::~verilogwriter() {
  //--
}

void verilogwriter::print(context* ctx) {
  // print header
  this->print_header(ctx);

  out_ << std::endl;

  // print body
  this->print_body(ctx);

  out_ << std::endl;

  // print footer
  this->print_footer(ctx);
}

void verilogwriter::print_header(context* ctx) {
  //
  // includes
  //
  std::string lib_file = "verilog.v";
  auto lib_dir = std::getenv("CASH_PATH");
  if (lib_dir) {
    lib_file = std::string(lib_dir) + "/" + lib_file;
  }
  out_ << "`include \"" << lib_file << "\"" << std::endl;

  out_ << std::endl;

  //
  // ports declaration
  //
  out_ << "module " << ctx->get_name() << '(';
  {
    auto_indent indent(out_);
    const char* sep = "";

    auto default_clk = ctx->get_default_clk();
    if (default_clk) {
      out_ << sep << std::endl; sep = ",";
      this->print_port(default_clk);
    }

    auto default_reset = ctx->get_default_reset();
    if (default_reset) {
      out_ << sep << std::endl; sep = ",";
      this->print_port(default_reset);
    }

    for (auto input : ctx->get_inputs()) {
      out_ << sep << std::endl; sep = ",";
      this->print_port(input);
    }

    for (auto output : ctx->get_outputs()) {
      out_ << sep << std::endl; sep = ",";
      this->print_port(output);
    }

    if (strlen(sep) != 0)
      out_ << std::endl;
  }
  out_ << ");" << std::endl;

  out_ << std::endl;

  //
  // signals declaration
  //
  {
    auto_indent indent(out_);

    for (auto node : ctx->get_nodes()) {
      this->print_decl(node);
    }
  }
}

void verilogwriter::print_body(context* ctx) {
  //
  // module logic
  //
  {
    auto_indent indent(out_);
    for (auto node : ctx->get_nodes()) {
      auto op = node->get_op();
      switch (op) {
      case op_lit:
        this->print_literal(dynamic_cast<litimpl*>(node));
        break;
      case op_proxy:
        this->print_proxy(dynamic_cast<proxyimpl*>(node));
        break;
      case op_alu:
        this->print_alu(dynamic_cast<aluimpl*>(node));
        break;
      case op_select:
        this->print_select(dynamic_cast<selectimpl*>(node));
        break;
      case op_reg:
        this->print_reg(dynamic_cast<regimpl*>(node));
        break;
      case op_mem:
        this->print_mem(dynamic_cast<memimpl*>(node));
        break;
      case op_input:
      case op_clk:
      case op_reset:
      case op_output:
      case op_tap:
      case op_memport:
      case op_assert:
      case op_print:
      case op_tick:
        break;
      default:
        assert(false);
      }
    }
  }
}

void verilogwriter::print_footer(context* ctx) {
  //
  // output assignment
  //
  {
    auto_indent indent(out_);
    for (auto output : ctx->get_outputs()) {
      out_ << "assign ";
      this->print_name(output);
      out_ << " = ";
      this->print_name(dynamic_cast<outputimpl*>(output)->get_src(0).get_impl());
      out_ << ";" << std::endl;
    }
  }
  out_ << std::endl;
  out_ << "endmodule" << std::endl;
}

void verilogwriter::print_port(lnodeimpl* node) {
  auto op = node->get_op();
  switch (op) {
  case op_input:
  case op_clk:
  case op_reset:
    out_ << "input";
    break;
  case op_output:
  case op_tap:
    out_ << "output";
    break;
  default:
    assert(false);
  }

  out_ << " ";
  this->print_type(node);

  out_ << " ";
  this->print_name(node);
}

void verilogwriter::print_decl(lnodeimpl* node) {
  auto op = node->get_op();
  switch (op) {
  case op_mem:
    this->print_type(node);
    out_ << " ";
    this->print_name(node);
    out_ << "[0:" << ((1 << dynamic_cast<memimpl*>(node)->get_addr_width()) - 1) << "]";
    out_ << ";" << std::endl;
    break;
  case op_lit:
  case op_proxy:
  case op_alu:
  case op_select:
  case op_reg:
    this->print_type(node);
    out_ << " ";
    this->print_name(node);
    out_ << ";" << std::endl;
    break;
  case op_input:
  case op_clk:
  case op_reset:
  case op_output:
  case op_tap:
  case op_memport:
  case op_assert:
  case op_print:
  case op_tick:
    break;
  default:
    assert(false);
  }
}

void verilogwriter::print_literal(litimpl* node) {
  out_ << "assign ";
  this->print_name(node);
  out_ << " = ";
  this->print_value(node->get_value());
  out_ << ";" << std::endl;
}

void verilogwriter::print_proxy(proxyimpl* node) {
  out_ << "assign ";
  this->print_name(node);
  out_ << " = ";
  const auto& ranges = node->get_ranges();
  uint32_t dst_offset = node->get_size();
  auto print_range = [&](const proxyimpl::range_t& range) {
    dst_offset -= range.length;
    assert(range.dst_offset == dst_offset);
    auto& src = node->get_src(range.src_idx);
    this->print_name(src.get_impl());
    if (range.length < src.get_size()) {
      out_ << "[";
      if (range.length > 1) {
        out_ << (range.src_offset + range.length - 1) << ":" << range.src_offset;
      } else {
        out_ << range.src_offset;
      }
      out_ << "]";
    };
  };
  if (ranges.size() > 1) {
    out_ << "{";
    const char* sep = "";
    for (int i = ranges.size() - 1; i >= 0; --i) {
      out_ << sep;
      sep = ", ";
      print_range(ranges[i]);
    }
    out_ << "}";
  } else {
    print_range(ranges[0]);
  }  
  out_ << ";" << std::endl;
}

void verilogwriter::print_alu(aluimpl* node) {
  auto alu_op = node->get_alu_op();
  if (alu_op == alu_op_rotl) {
    this->print_rotate(node, false);
  } else
  if (alu_op == alu_op_rotr) {
    this->print_rotate(node, true);
  } else
  if (alu_op == alu_op_mux) {
    this->print_mux(node);
  } else
  if (CH_ALUOP_DATA(alu_op) == alu_integer) {
    if (CH_ALUOP_ARY(alu_op) == alu_binary) {
      out_ << "assign ";
      this->print_name(node);
      out_ << " = ";
      this->print_name(node->get_src(0).get_impl());
      out_ << " ";
      this->print_operator(alu_op);
      out_ << " ";
      this->print_name(node->get_src(1).get_impl());
      out_ << ";" << std::endl;
    } else {
      assert(CH_ALUOP_ARY(alu_op) == alu_unary);
      out_ << "assign ";
      this->print_name(node);
      out_ << " = ";
      this->print_operator(alu_op);
      this->print_name(node->get_src(0).get_impl());
      out_ << ";" << std::endl;
    }
  } else {
    if (alu_op == alu_op_fmult) {
      this->print_fmult(node);
    } else
    if (alu_op == alu_op_fadd) {
      this->print_fadd(node);
    } else {
      CH_TODO();
    }
  }
}

void verilogwriter::print_rotate(aluimpl* node, bool right_dir) {
  out_ << "barrel_shift #(";
  out_ << (right_dir ? 1 : 0);
  out_ << ", ";
  out_ << node->get_size();
  out_ << ") __barrel_shift_";
  out_ << node->get_id() << "__(";
  this->print_name(node->get_src(0).get_impl());
  out_ << ", ";
  this->print_name(node);
  out_ << ", ";
  this->print_name(node->get_src(1).get_impl());
  out_ << ");" << std::endl;
}

void verilogwriter::print_mux(aluimpl* node) {
  out_ << "bus_mux #(";
  out_ << node->get_size();
  out_ << ", ";
  out_ << node->get_src(1).get_size();
  out_ << ") __bus_mux_";
  out_ << node->get_id() << "__(";
  this->print_name(node->get_src(0).get_impl());
  out_ << ", ";
  this->print_name(node->get_src(1).get_impl());
  out_ << ", ";
  this->print_name(node);
  out_ << ");" << std::endl;
}

void verilogwriter::print_fmult(aluimpl* node) {
  out_ << "fp_mult __fp_mult_";
  out_ << node->get_id() << "__(.clock(clk), .dataa(";
  this->print_name(node->get_src(0).get_impl());
  out_ << "), .datab(";
  this->print_name(node->get_src(1).get_impl());
  out_ << "), .result(";
  this->print_name(node);
  out_ << "));" << std::endl;
}

void verilogwriter::print_fadd(aluimpl* node) {
  out_ << "fp_add __fp_add_sub_";
  out_ << node->get_id() << "__(.clock(clk), .dataa(";
  this->print_name(node->get_src(0).get_impl());
  out_ << "), .datab(";
  this->print_name(node->get_src(1).get_impl());
  out_ << "), .result(";
  this->print_name(node);
  out_ << "));" << std::endl;
}

void verilogwriter::print_select(selectimpl* node) {
  out_ << "assign ";
  this->print_name(node);
  out_ << " = ";
  this->print_name(node->get_pred().get_impl());
  out_ << " ? ";
  this->print_name(node->get_true().get_impl());
  out_ << " : ";
  this->print_name(node->get_false().get_impl());
  out_ << ";" << std::endl;
}

void verilogwriter::print_reg(regimpl* node) {
  auto assign_value = [&](const lnode& value) {
    this->print_name(node);
    out_ << " <= ";
    this->print_name(value.get_impl());
    out_ << ";" << std::endl;
  };
  out_ << "always @ (";
  this->print_cdomain(node->get_cd());
  out_ << ")" << std::endl;
  out_  << "begin" << std::endl;
  {
    auto_indent indent(out_);
    if (node->has_reset()) {
      out_ << "if (";
      this->print_name(node->get_reset().get_impl());
      out_ << ")" << std::endl;
      {
        auto_indent indent(out_);
        assign_value(node->get_init());
      }
      if (node->has_enable()) {
        out_ << "else if (";
        this->print_name(node->get_enable().get_impl());
        out_ << ")" << std::endl;
        {
          auto_indent indent(out_);
          assign_value(node->get_next());
        }
      } else {
        out_ << "else" << std::endl;
        {
          auto_indent indent(out_);
          assign_value(node->get_next());
        }
      }
    } else
    if (node->has_enable()) {
      out_ << "if (";
      this->print_name(node->get_enable().get_impl());
      out_ << ")" << std::endl;
      {
        auto_indent indent(out_);
        assign_value(node->get_next());
      }
    } else {
      assign_value(node->get_next());
    }
  }
  out_ << "end" << std::endl;
}

void verilogwriter::print_cdomain(cdomain* cd) {
  const char* sep = "";
  for (auto& e : cd->get_sensitivity_list()) {
    out_ << sep;
    sep = ", ";
    switch (e.get_edgedir()) {
    case EDGE_POS:
      out_ << "posedge ";
      break;
    case EDGE_NEG:
      out_ << "negedge ";
      break;
    default:
      break;
    }
    this->print_name(e.get_signal().get_impl());
  }
}

void verilogwriter::print_mem(memimpl* node) {
  //
  // initialization data
  //
  if (node->is_initialized()) {
    out_ << "initial begin" << std::endl;
    {
      auto_indent indent(out_);
      const auto& value = node->get_value();
      uint32_t data_width = node->get_data_width();
      uint32_t addr_size = 1 << node->get_addr_width();
      for (uint32_t addr = 0; addr < addr_size; ++addr) {
        this->print_name(node);
        out_ << "[" << addr << "] = " << data_width << "'b";
        uint32_t data_msb = (addr + 1) * data_width - 1;
        auto it = value.begin() + data_msb;
        for (uint32_t n = data_width; n--;) {
          out_ << ((*it--) ? 1 : 0);
        }
        out_ << ";" << std::endl;
      }
    }
    out_ << "end" << std::endl;
  }

  //
  // write ports logic
  //
  for (auto& p : node->get_ports()) {
    auto p_impl = dynamic_cast<memportimpl*>(p.get_impl());
    if (!p_impl->is_writable())
      continue;
    out_ << "always @(" ;
    this->print_cdomain(node->get_cd());
    out_ << ")" << std::endl;
    out_ << "begin" << std::endl;
    {
      auto_indent indent(out_);
      this->print_name(p_impl);
      out_ << " = ";
      this->print_name(p_impl->get_wdata().get_impl());
      out_ << ";" << std::endl;
    }
    out_ << "end" << std::endl;
  }
}

void verilogwriter::print_operator(ch_alu_op alu_op) {
  switch (alu_op) {
  case alu_op_inv: out_ << "~"; break;
  case alu_op_and: out_ << "&"; break;
  case alu_op_or:  out_ << "|"; break;
  case alu_op_xor: out_ << "^"; break;
  case alu_op_nand: out_ << "~&"; break;
  case alu_op_nor:  out_ << "~|"; break;
  case alu_op_xnor: out_ << "~^"; break;
  case alu_op_andr: out_ << "&"; break;
  case alu_op_orr:  out_ << "|"; break;
  case alu_op_xorr: out_ << "^"; break;
  case alu_op_nandr: out_ << "~&"; break;
  case alu_op_norr:  out_ << "~|"; break;
  case alu_op_xnorr: out_ << "~^"; break;
  case alu_op_neg: out_ << "-"; break;
  case alu_op_add: out_ << "+"; break;
  case alu_op_sub: out_ << "-"; break;
  case alu_op_mult:out_ << "*"; break;
  case alu_op_div: out_ << "/"; break;
  case alu_op_mod: out_ << "%"; break;
  case alu_op_sll: out_ << "<<"; break;
  case alu_op_srl: out_ << ">>"; break;
  case alu_op_sra: out_ << ">>>"; break;
  case alu_op_eq:  out_ << "=="; break;
  case alu_op_ne:  out_ << "!="; break;
  case alu_op_lt:  out_ << "<"; break;
  case alu_op_gt:  out_ << ">"; break;
  case alu_op_le:  out_ << "<="; break;
  case alu_op_ge:  out_ << ">="; break;
  default:
    assert(false);
  }
}

void verilogwriter::print_name(lnodeimpl* node) {
  auto print_basic_name = [&](char prefix) {
    out_ << "__" << prefix;
    out_ << node->get_id();
    out_ << "__";
  };
  auto op = node->get_op();
  switch (op) {
  case op_clk:
  case op_reset:
  case op_input:
  case op_output:
    out_ << dynamic_cast<ioimpl*>(node)->get_name();
    break;
  case op_proxy:
    print_basic_name('w');
    break;
  case op_lit:
    print_basic_name('l');
    break;
  case op_select:
    print_basic_name('s');
    break;
  case op_alu:
    print_basic_name('a');
    break;
  case op_reg:
    print_basic_name('r');
    break;
  case op_mem:
    print_basic_name('m');
    break;
  case op_memport:
    out_ << "__m";
    out_ << dynamic_cast<memportimpl*>(node)->get_mem().get_id();
    out_ << "__[";
    this->print_name(dynamic_cast<memportimpl*>(node)->get_addr().get_impl());
    out_ << "]";
    break;
  default:
    assert(false);
  }
}

void verilogwriter::print_type(lnodeimpl* node) {
  auto op = node->get_op();
  switch (op) {
  case op_reg:
  case op_mem:
    out_ << "reg";
    break;
  default:
    out_ << "wire";
    break;
  }
  if (op == op_mem) {
    auto data_width = dynamic_cast<memimpl*>(node)->get_data_width();
    if (data_width > 1) {
      out_ << "[" << (data_width - 1) << ":0]";
    }
  } else {
    if (node->get_size() > 1) {
      out_ << "[" << (node->get_size() - 1) << ":0]";
    }
  }
}

void verilogwriter::print_value(const bitvector& value) {
  out_ << value.get_size() << "'b";
  for (auto it = value.rbegin(), end = value.rend(); it != end;) {
    out_ << ((*it++) ? 1 : 0);
  }
}
