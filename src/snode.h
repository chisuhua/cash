#pragma once

#include "typebase.h"
#include "bitvector.h"

namespace cash {
namespace detail {

class snodeimpl;

class snode {
public:
  
  using data_type = nodeset<snodeimpl*>;
  
  snode();

  snode(const snode& rhs);

  snode(uint32_t size, const snode& rhs);

  snode(snode&& rhs);

  explicit snode(snodeimpl* impl);

  snode(const data_type& data);

  snode(const bitvector& value);

  ~snode();
  
  snodeimpl* get_impl() const;
  
  snode& operator=(const snode& rhs);
  
  snode& operator=(snode&& rhs);
  
  bool is_equal(const snode& rhs, uint32_t size) const;
  
  bool is_less(const snode& rhs, uint32_t size) const;
  
  uint32_t get_size() const;
  
  uint32_t read(uint32_t idx, uint32_t size) const;
  
  void write(uint32_t idx, uint32_t value, uint32_t size);
  
  void read(uint8_t* out,
            uint32_t offset,
            uint32_t length,
            uint32_t size) const;
  
  void write(const uint8_t* in,
             uint32_t offset,
             uint32_t length,
             uint32_t size);

  void assign(uint32_t size, const snode& rhs);

  void assign(const bitvector& value);

  void read_data(data_type& inout,
                 uint32_t offset,
                 uint32_t length,
                 uint32_t size) const;
  
  void write_data(uint32_t dst_offset,
                  const data_type& in,
                  uint32_t src_offset,
                  uint32_t src_length,
                  uint32_t size);

protected:

  void ensureInitialized(uint32_t size) const;

  void clear();
  
  void assign(uint32_t dst_offset,
              snodeimpl* src,
              uint32_t src_offset,
              uint32_t src_length,
              uint32_t size);
  
  void assign(snodeimpl* impl, bool is_owner);
  
  void move(snode& rhs);
  
  void clone(bool keep_data);
    
  mutable snodeimpl* impl_;
};

std::ostream& operator<<(std::ostream& os, const snode& node);

}
}
