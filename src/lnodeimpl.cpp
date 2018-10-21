#include "lnodeimpl.h"
#include "proxyimpl.h"
#include "regimpl.h"
#include "memimpl.h"
#include "udfimpl.h"
#include "context.h"

using namespace ch::internal;

const char* ch::internal::to_string(lnodetype type) {
  static const char* sc_names[] = {
    CH_LNODE_ENUM(CH_LNODE_NAME)
  };
  return sc_names[CH_LNODE_INDEX(type)];
}

cdimpl* ch::internal::get_snode_cd(lnodeimpl* node) {
  switch (node->type()) {
  case type_reg:
    return reinterpret_cast<cdimpl*>(
          reinterpret_cast<regimpl*>(node)->cd().impl());
  case type_msrport:
    return reinterpret_cast<cdimpl*>(
          reinterpret_cast<memportimpl*>(node)->cd().impl());
  case type_mwport:
    return reinterpret_cast<cdimpl*>(
          reinterpret_cast<memportimpl*>(node)->cd().impl());
  case type_udfs:
    return reinterpret_cast<cdimpl*>(
          reinterpret_cast<udfsimpl*>(node)->cd().impl());
  default:
    std::abort();
  }
  return nullptr;
}

lnodeimpl::lnodeimpl(context* ctx,
                     lnodetype type,
                     uint32_t size,                     
                     const source_location& sloc,
                     const std::string& name,
                     uint32_t var_id)
  : ctx_(ctx)
  , id_(ctx->node_id())
  , type_(type)
  , size_(size)
  , sloc_(sloc)
  , name_(to_string(type))
  , var_id_(var_id) {
  if (!name.empty()) {
    name_ = name;
  }
}

lnodeimpl::~lnodeimpl() {}

uint32_t lnodeimpl::add_src(const lnode& src) {
  uint32_t index = srcs_.size();
  srcs_.push_back(src);
  return index;
}

bool lnodeimpl::equals(const lnodeimpl& other) const {
  if (this->type() == other.type()
   && this->size() == other.size()
   && this->srcs().size() == other.srcs().size()) {
    for (unsigned i = 0, n = this->srcs().size(); i < n; ++i) {
      if (this->src(i).id() != other.src(i).id())
        return false;
    }
    return true;
  }
  return false;
}

lnodeimpl* lnodeimpl::slice(uint32_t offset,
                            uint32_t length,
                            const source_location& sloc) {
  assert(length <= size_);
  if (size_ == length)
    return this;
  return ctx_->create_node<proxyimpl>(this, offset, length, sloc);
}

void lnodeimpl::write(uint32_t,
                      const lnode&,
                      uint32_t,
                      uint32_t,
                      const source_location&) {
  assert(false);
}

void lnodeimpl::print(std::ostream& out) const {
  out << "#" << id_ << " <- " << this->type() << size_;
  out << "(";
  for (uint32_t i = 0; i < srcs_.size(); ++i) {
    if (i)
      out << ", ";
    out << "#" << srcs_[i].id();
  }
  out << ")";
}

std::string lnodeimpl::debug_info() const {
  return stringf("%s%d (#%d) (@var%d) in module '%s (#%d)'  (%s:%d)",
                 name_.c_str(),
                 size_,
                 id_,
                 var_id_,
                 ctx_->name().c_str(),
                 ctx_->id(),
                 sloc_.file(),
                 sloc_.line());
}

///////////////////////////////////////////////////////////////////////////////

undefimpl::undefimpl(context* ctx, uint32_t size, const source_location& sloc)
  : lnodeimpl(ctx, type_undef, size, sloc)
{}
