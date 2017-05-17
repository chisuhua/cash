#pragma once

#include "snode.h"

namespace chdl_internal {

class snodeimpl : public refcounted {
public:
  snodeimpl(uint32_t size);
  snodeimpl(const bitvector& value);
  ~snodeimpl();  
  
  uint32_t get_id() const {
    return m_id;
  }
  
  bool isOwner(snode* node) const {
    return (m_owner == node);
  }
  
  void setOwnership(snode* node);
  
  void clearOwnership(snode* node);
  
  void assign(uint32_t start, snodeimpl* src, uint32_t offset, uint32_t length);

  bitvector::const_reference operator[](uint32_t idx) const {
    this->sync_sources();
    return m_value[idx];
  }
  
  bitvector::reference operator[](uint32_t idx) {
    ++m_changeid;
    this->sync_sources();
    return m_value[idx];
  }
  
  uint32_t read(uint32_t idx) const {
    this->sync_sources();
    return m_value.get_word(idx);
  }
  
  void write(uint32_t idx, uint32_t value) {
    this->sync_sources();
    m_value.set_word(idx, value);
    ++m_changeid;
  }
  
  void readBytes(uint8_t* out, uint32_t sizeInBytes) const {
    this->sync_sources();
    m_value.readBytes(out, sizeInBytes);
  }
  
  void writeBytes(const uint8_t* in, uint32_t sizeInBytes) {
    this->sync_sources();
    m_value.writeBytes(in, sizeInBytes);
    ++m_changeid;
  }
  
  const bitvector& read() const {
    this->sync_sources();
    return m_value;
  }
  
  void write(const bitvector& value) {
    assert(m_srcs.size() == 0);
    m_value = value;
    ++m_changeid;
  }
  
  uint32_t get_size() const {
    return m_value.get_size();
  }
  
  const bitvector& get_value() const { 
    this->sync_sources();
    return m_value;
  }
  
  bitvector& get_value() {
    this->sync_sources();
    return m_value;
  }
  
protected:
  
  uint64_t sync_sources() const;
  
  void merge_left(uint32_t idx);
  
  struct source_t {
    snodeimpl* node;
    uint32_t start;    
    uint32_t offset;    
    uint32_t length;
    uint64_t changeid;
  };
  
  mutable std::vector<source_t> m_srcs;
  mutable bitvector m_value;
  mutable uint64_t  m_changeid;
  snode* m_owner;
  uint32_t m_id;
};

}
