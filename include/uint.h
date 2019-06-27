#pragma once

#include "numberbase.h"

namespace ch {
namespace internal {

template <unsigned N = 32>
class ch_suint : public ch_snumber_base<ch_suint<N>> {
public:
  static_assert(N != 0, "invalid size");
  using traits = system_traits<N, false, ch_suint, ch_uint<N>>;
  using base = ch_snumber_base<ch_suint<N>>;
  using base::operator=;

  ch_suint(const system_buffer& buffer
           = make_system_buffer(N, idname<ch_suint>()))
    : buffer_(buffer) {
    assert(N == buffer->size());
  }

  template <typename U,
            CH_REQUIRE(std::is_integral_v<U>)>
  ch_suint(const U& other)
    : ch_suint(make_system_buffer(N, idname<ch_suint>())) {
    this->operator=(other);
  }

  template <typename U,
            CH_REQUIRE(is_bitvector_extended_type_v<U>)>
  explicit ch_suint(U&& other)
    : ch_suint(make_system_buffer(N, idname<ch_suint>())) {
    this->operator=(make_system_buffer(sdata_type(N , std::forward<U>(other))));
  }

  template <typename U,
            CH_REQUIRE(ch_width_v<U> <= N)>
  explicit ch_suint(const ch_sbit_base<U>& other)
    : ch_suint(make_system_buffer(N, idname<ch_suint>())) {
    this->operator=(reinterpret_cast<const U&>(other));
  }

  template <unsigned M,
            CH_REQUIRE(M <= N)>
  ch_suint(const ch_sbit<M>& other)
    : ch_suint(make_system_buffer(N, idname<ch_suint>())) {
    this->operator=(other);
  }

  template <unsigned M,
            CH_REQUIRE(M < N)>
  ch_suint(const ch_sint<M>& other)
    : ch_suint(make_system_buffer(N, idname<ch_suint>())) {
    this->operator=(other);
  }

  template <unsigned M,
            CH_REQUIRE(M < N)>
  ch_suint(const ch_suint<M>& other)
    : ch_suint(make_system_buffer(N, idname<ch_suint>())) {
    this->operator=(other);
  }

  ch_suint(const ch_suint& other)
    : ch_suint(system_accessor::copy(other)) {
    this->operator=(other);
  }

  ch_suint(ch_suint&& other) : buffer_(std::move(other.buffer_)) {}

  ch_suint& operator=(const ch_suint& other) {
    system_accessor::assign(*this, other);
    return *this;
  }

  ch_suint& operator=(ch_suint&& other) {
    system_accessor::move(*this, std::move(other));
    return *this;
  }

protected:

  const system_buffer& __buffer() const {
    return buffer_;
  }

  system_buffer buffer_;

  friend class system_accessor;
};

///////////////////////////////////////////////////////////////////////////////

template <unsigned N = 32>
class ch_uint : public ch_number_base<ch_uint<N>> {
public:  
  static_assert(N != 0, "invalid size");
  using traits = logic_traits<N, false, ch_uint, ch_suint<N>>;
  using base = ch_number_base<ch_uint<N>>;
  using base::operator=;

  ch_uint(const logic_buffer& buffer = make_logic_buffer(N, idname<ch_uint>()))
    : buffer_(buffer) {
    assert(N == buffer.size());
  }

  template <typename U,
            CH_REQUIRE(std::is_integral_v<U>)>
  ch_uint(const U& other)
    : ch_uint(make_logic_buffer(N, idname<ch_uint>())) {
    CH_SOURCE_LOCATION(1);
    this->operator=(other);
  }

  template <typename U,
            CH_REQUIRE(ch_width_v<U> <= N)>
  explicit ch_uint(const ch_sbit_base<U>& other)
    : ch_uint(make_logic_buffer(N, idname<ch_uint>())) {
    CH_SOURCE_LOCATION(1);
    this->operator=(reinterpret_cast<const U&>(other));
  }

  template <unsigned M,
            CH_REQUIRE(M <= N)>
  ch_uint(const ch_sbit<M>& other)
    : ch_uint(make_logic_buffer(N, idname<ch_uint>())) {
    CH_SOURCE_LOCATION(1);
    this->operator=(other);
  }

  template <unsigned M,
            CH_REQUIRE(M <= N)>
  ch_uint(const ch_sint<M>& other)
    : ch_uint(make_logic_buffer(N, idname<ch_uint>())) {
    CH_SOURCE_LOCATION(1);
    this->operator=(other);
  }

  template <unsigned M,
            CH_REQUIRE(M <= N)>
  ch_uint(const ch_suint<M>& other)
    : ch_uint(make_logic_buffer(N, idname<ch_uint>())) {
    CH_SOURCE_LOCATION(1);
    this->operator=(other);
  }

  template <typename U,
            CH_REQUIRE(ch_width_v<U> <= N)>
  explicit ch_uint(const ch_bit_base<U>& other)
    : ch_uint(make_logic_buffer(N, idname<ch_uint>())) {
    CH_SOURCE_LOCATION(1);
    this->operator=(reinterpret_cast<const U&>(other));
  }

  template <unsigned M,
            CH_REQUIRE(M <= N)>
  ch_uint(const ch_bit<M>& other)
    : ch_uint(make_logic_buffer(N, idname<ch_uint>())) {
    CH_SOURCE_LOCATION(1);
    this->operator=(other);
  }

  template <unsigned M,
            CH_REQUIRE(M < N)>
  ch_uint(const ch_int<M>& other)
    : ch_uint(make_logic_buffer(N, idname<ch_uint>())) {
    CH_SOURCE_LOCATION(1);
    this->operator=(other);
  }

  template <unsigned M,
            CH_REQUIRE(M < N)>
  ch_uint(const ch_uint<M>& other)
    : ch_uint(make_logic_buffer(N, idname<ch_uint>())) {
    CH_SOURCE_LOCATION(1);
    this->operator=(other);
  }

  ch_uint(const ch_uint& other)
    : ch_uint(make_logic_buffer(N, idname<ch_uint>())) {
    CH_SOURCE_LOCATION(1);
    this->operator=(other);
  }

  ch_uint(ch_uint&& other) : buffer_(std::move(other.buffer_)) {}

  ch_uint& operator=(const ch_uint& other) {
    CH_SOURCE_LOCATION(1);
    logic_accessor::assign(*this, other);
    return *this;
  }

  ch_uint& operator=(ch_uint&& other) {
    CH_SOURCE_LOCATION(1);
    logic_accessor::move(*this, std::move(other));
    return *this;
  }

protected:

  const logic_buffer& __buffer() const {
    return buffer_;
  }

  logic_buffer buffer_;

  friend class logic_accessor;
};

}}
