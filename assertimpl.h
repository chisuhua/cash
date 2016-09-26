#pragma once

#include "ioimpl.h"

namespace chdl_internal {

class assertimpl : public ioimpl {
public:
  assertimpl(const ch_node& src, const std::string& msg);
  ~assertimpl() {}
  
  const bitvector& eval(ch_cycle t);

private:
  std::string m_msg;
};

}
