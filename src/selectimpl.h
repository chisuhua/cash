#pragma once

#include "lnodeimpl.h"

namespace ch {
namespace internal {

class selectimpl : public lnodeimpl {
public:

  bool has_key() const {
    return has_key_;
  }

  const bitvector& eval(ch_tick t) override;

  void print(std::ostream& out, uint32_t level) const override;

protected:

  selectimpl(context* ctx, uint32_t size, lnodeimpl* key);

  selectimpl(context* ctx,
             const lnode& pred,
             const lnode& _true,
             const lnode& _false);

  ~selectimpl() {}

  ch_tick tick_;

  bool has_key_;

  friend class context;
};

}
}
