#pragma once

namespace chdl_internal {

class lnode;
class snode;
class lnodeimpl;
class snodeimpl;
class undefimpl;
class litimpl;
class ioimpl;
class inputimpl;
class outputimpl;
class tapimpl;
class assertimpl;
class clock_event;
class cdomain;

class context : public refcounted {
public:
  context();
  ~context();

  //--

  void push_clk(lnodeimpl* clk);
  void pop_clk();
  lnodeimpl* get_clk();

  void push_reset(lnodeimpl* reset);
  void pop_reset();
  lnodeimpl* get_reset();

  //--
  
  uint32_t add_node(lnodeimpl* node);  
  void remove_node(lnodeimpl* node);
  
  void begin_branch();
  void end_branch();
  
  void begin_case(lnodeimpl* cond);
  void end_case();
  bool conditional_enabled(lnodeimpl* node = nullptr) const;
  lnodeimpl* resolve_conditional(lnodeimpl* dst, lnodeimpl* src);
  
  litimpl* create_literal(const bitvector& value);
  
  cdomain* create_cdomain(const std::vector<clock_event>& sensitivity_list);
  void remove_cdomain(cdomain* cd);
    
  //-- 

  lnodeimpl* bind_input(snodeimpl* bus);  
  snodeimpl* bind_output(lnodeimpl* output);
  
  void register_tap(const std::string& name, lnodeimpl* lnode);
  snodeimpl* get_tap(const std::string& name, uint32_t size);
  
  //--
  
  void syntax_check();
    
  //--
  
  void get_live_nodes(std::set<lnodeimpl*>& live_nodes);
  
  //--
  
  void tick(ch_cycle t);  
  void tick_next(ch_cycle t);
  void eval(ch_cycle t);
  
  //--
  
  void toVerilog(const std::string& module_name, std::ostream& out);
  
  void dumpAST(std::ostream& out, uint32_t level);
  
  void dumpCFG(lnodeimpl* node, std::ostream& out, uint32_t level);
  
  void dump_stats(std::ostream& out);
  
protected:
  
  struct cond_val_t {
    lnodeimpl* dst;
    lnodeimpl* sel;
    lnodeimpl* owner;
  };
  
  struct cond_case_t {
    lnodeimpl* cond;
    std::set<lnodeimpl*> locals;
    std::vector<uint32_t> assignments; 
    cond_case_t(lnodeimpl* cond_) : cond(cond_) {}
  };
  
  struct cond_block_t {
    std::list<cond_case_t> cases;
  };
  
  typedef std::vector<cond_val_t> cond_vals_t;
  typedef std::list<cond_block_t> cond_blocks_t;
  
  lnodeimpl* get_current_conditional(const cond_blocks_t::iterator& iterBlock, lnodeimpl* dst) const;
  
  std::list<lnodeimpl*>   m_nodes;
  std::list<undefimpl*>   m_undefs;  
  std::list<inputimpl*>   m_inputs;
  std::list<outputimpl*>  m_outputs;
  std::list<tapimpl*>     m_taps;
  std::list<ioimpl*>      m_gtaps;
  std::list<litimpl*>     m_literals;
  std::list<cdomain*>     m_cdomains;
  cond_blocks_t           m_cond_blocks;   
  cond_vals_t             m_cond_vals;
  std::stack<lnode>       m_clk_stack;
  std::stack<lnode>       m_reset_stack;

  uint32_t   m_nodeids;
  inputimpl* m_clk;
  inputimpl* m_reset;
  
  std::map<std::string, unsigned> m_dup_taps;
  
  friend class ch_compiler;
  friend class ch_simulator;
  friend class ch_tracer;
};

context* ctx_begin();
context* ctx_curr();
void ctx_end();

}
