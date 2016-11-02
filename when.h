#pragma once

#include "bitv.h"

namespace chdl_internal {

class when_t {
public:
  
  ~when_t() {
    if (m_stmts) {
      CHDL_CHECK(m_stmts->empty(), "incomplete when statement");
      delete m_stmts;
    }
  }
    
  template <typename Func>
  when_t& _when(const ch_logicbase& cond, const Func& func) {
    m_stmts->push({cond.get_node().get_impl(), to_function(func)});
    return *this; 
  }
  
  template <typename Func>
  void _else(const Func& func) {
    func(); // evaluate 'else' case
    this->eval();
  }
  
  void _end() {
    this->eval();
  }
  
protected:
  
  using func_t = std::function<void ()>;
  
  struct stmt_t {
    lnodeimpl* cond;
    func_t func;
  };
  
  using stmts_t = std::stack<stmt_t>;
  
  when_t(lnodeimpl* cond, func_t func) : m_stmts(new stmts_t()) {
    m_stmts->push({cond, func});
  }
  
  void eval();
  
  stmts_t* m_stmts;
    
  template <typename Func> 
  friend when_t ch_when(const ch_logicbase& cond, const Func& func);
};

template <typename Func> 
when_t ch_when(const ch_logicbase& cond, const Func& func) {
  return when_t(cond.get_node().get_impl(), to_function(func));
}

}
