#pragma once

#include "typebase.h"
#include "bitvector.h"

namespace chdl_internal {

class lnodeimpl;
class context;

using ch_cycle = uint64_t;

class lnode {
public:
  
  using bitstream_type = bitstream<lnode>;

  lnode() : m_impl(nullptr) {}
  lnode(uint32_t size);  
  lnode(const lnode& rhs);
  lnode(lnode&& rhs);
  explicit lnode(lnodeimpl* impl);
  lnode(const bitstream_type& data);
  lnode(const bitvector& value);
  
  ~lnode();
  
  const lnode& ensureInitialized(uint32_t size) const;
  
  lnodeimpl* get_impl() const;  
  
  lnode& operator=(const lnode& rhs);
  
  lnode& operator=(lnode&& rhs);
  
  lnode& operator=(lnodeimpl* rhs);
  
  uint32_t get_id() const;
  
  uint32_t get_size() const;
  
  context* get_ctx() const;
  
  bool ready() const;
  
  bool valid() const;
 
  const bitvector& eval(ch_cycle t);  
  
  void assign(const bitvector& value);
  
  void read(bitstream_type& inout, uint32_t offset, uint32_t length, uint32_t size) const;
  
  void write(uint32_t dst_offset, const bitstream_type& in, uint32_t src_offset, uint32_t src_length, uint32_t size);

protected:   
   
  void clear() const;

  void assign(lnodeimpl* impl, const lnode* src, bool initialization = false) const;
  
  void move(lnode& rhs);
  
  void assign(uint32_t dst_offset, const lnode& src, uint32_t src_offset, uint32_t src_length, uint32_t size, bool initialization = false);
  
  mutable lnodeimpl* m_impl;
  
  friend class lnodeimpl;
  friend class context; 
};

inline std::ostream& operator<<(std::ostream& out, const lnode& rhs) {
  out << rhs.get_id();
  return out;
}

}
