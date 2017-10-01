#include "litimpl.h"
#include "context.h"

using namespace cash::internal;
litimpl::litimpl(context* ctx, uint32_t size)
  : lnodeimpl(op_lit, ctx, size) {}

litimpl::litimpl(context* ctx, const bitvector& value) 
  : lnodeimpl(op_lit, ctx, value.get_size()) {
  value_ = value;
}

const bitvector& litimpl::eval(ch_tick) { 
  return value_; 
}

void litimpl::print(std::ostream& out, uint32_t level) const {
  CH_UNUSED(level);
  out << "#" << id_ << " <- " << this->get_op() << value_.get_size()
      << "(" << value_ << ")";
}
