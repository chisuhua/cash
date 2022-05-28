#pragma once

#include "../core.h"
#include <math.h>
#include "ieee754.hpp"

namespace ch {
namespace htl {
using namespace logic;
using namespace extension;

// |1|<- N-M-1 bits ->|<--- M bits -->|
// |S|EEEEEEEEEEEEEEEE|MMMMMMMMMMMMMMM|
//
template <unsigned M, unsigned E, unsigned TF>
class ch_float;

template <unsigned M, unsigned E, unsigned TF=0>
class ch_sfloat : public ch_snumbase<ch_sfloat<M, E, TF>> {
public:
  using ufloat_t = IEEE754<M, E, TF>;
  using uprimitive_t = typename ufloat_t::primitive;
  static constexpr unsigned N = ufloat_t::BITS;
  using traits = system_traits<N, true, ch_sfloat, ch_float<M, E, TF>>;
  using base = ch_snumbase<ch_sfloat<M, E, TF>>;
  using base::operator=;

  explicit ch_sfloat(const system_buffer& buffer = make_system_buffer(N))
    : buffer_(buffer)
  {}

  explicit ch_sfloat(const float& other)
    : ch_sfloat(make_system_buffer(N)) {
    this->operator=(other);
  }

  explicit ch_sfloat(float&& other)
    : ch_sfloat(make_system_buffer(N)) {
    this->operator=(other);
  }
/*
  explicit ch_sfloat(double&& other)
    : ch_sfloat(make_system_buffer(N)) {
    this->operator=(other);
  }
*/
  template <typename U,
            CH_REQUIRES(ch_width_v<U> <= N)>
  explicit ch_sfloat(const ch_sbitbase<U>& other)
    : ch_sfloat(make_system_buffer(N)) {
    base::operator=(reinterpret_cast<const U&>(other));
  }

  template <typename U,
            CH_REQUIRES(std::is_integral_v<U>)>
  ch_sfloat(const U& other)
    : ch_sfloat(make_system_buffer(N)) {
    this->operator=(other);
  }

  ch_sfloat(const ch_sfloat& other)
    : ch_sfloat(make_system_buffer(N)) {
    this->operator=(other);
  }

  ch_sfloat(const ufloat_t& other)
    : ch_sfloat(make_system_buffer(N)) {
    this->operator=(other);
  }

  ch_sfloat(ufloat_t&& other)
    : ch_sfloat(make_system_buffer(N)) {
    this->operator=(other);
  }

  template <unsigned U,
            CH_REQUIRES(U == N)>
  ch_sfloat(const ch_sbit<U>& other)
    : ch_sfloat(make_system_buffer(N)) {
    this->operator=(other);
  }

  template <unsigned U,
            CH_REQUIRES(U == N)>
  ch_sfloat(const ch_suint<U>& other)
    : ch_sfloat(make_system_buffer(N)) {
    this->operator=(other);
  }

  template <unsigned U,
            CH_REQUIRES(U == N)>
  ch_sfloat(const ch_sint<U>& other)
    : ch_sfloat(make_system_buffer(N)) {
    this->operator=(other);
  }

  ch_sfloat(ch_sfloat&& other) : buffer_(std::move(other.buffer_)) {}

  ch_sfloat& operator=(const ch_sfloat& other) {
    system_accessor::assign(*this, other);
    return *this;
  }

  ch_sfloat& operator=(ch_sfloat&& other) {
    system_accessor::move(*this, std::move(other));
    return *this;
  }

  ch_sfloat& operator=(const ufloat_t& other) {
    *this = other.data;
    return *this;
  }

  ch_sfloat& operator=(ufloat_t&& other) {
    *this = std::move(other.data);
    return *this;
  }

  ch_sfloat& operator=(float&& other) {
    static_assert(M == 23, "invalid ufloat size");
    static_assert(E == 8, "invalid ufloat size");
    ufloat_t ufloat = ufloat_t(other);
    *this = ufloat.data;
    /*
    mant() = ufloat.mantissa;
    exp() = ufloat.exponent;
    sign() = ufloat.sign;
    // ch_println("sfloat this={}, mant={}, exp={}, sign={}", *this, mant(), exp(), sign());
    CH_DBG(1, "sfloat mant=%x, exp=%x, sign=%x\n", mant(), exp(), sign());
    CH_DBG(1, "sfloat this %x\n", *this);
    */
    //ch_println("sfloat this={}", *this);
    return *this;
  }
/*
  ch_sfloat& operator=(double&& other) {
    static_assert(M == 52, "invalid ufloat size");
    static_assert(E == 11, "invalid ufloat size");
    ufloat_t ufloat = ufloat_t(other);
    *this = ufloat.data;
    return *this;
  }
*/
  // ------------------------- Cast operators ------------------------- //
  //		 Convert to another floating point value
  template <
			typename T,
			typename = typename std::enable_if<std::is_floating_point<T >::value, T >::type
  >
  explicit operator T() const {
    ufloat_t ufloat = ufloat_t::from_data(static_cast<uprimitive_t>(this->as_bit()));
    return static_cast<T>(ufloat);
  }


  ch_sbit<M> mant() const {
    return ch_sliceref<E>(this->as_bit(), 0);
  }

  ch_sbit<E> exp() const {
    return ch_sliceref<E>(this->as_bit(), M);
  }

  ch_sbit<1> sign() const {
    return ch_sliceref<1>(this->as_bit(), M + E);
  }


protected:

  template <typename R>
  auto do_neg() const {
    static_assert(std::is_same_v<R, ch_sfloat>, "invalid type");
    auto value = static_cast<ufloat_t>(*this);
    return ch_sfloat(0.0f - value);
  }

  template <typename R, typename U>
  auto do_add(const U& other) const {
    static_assert(std::is_same_v<R, ch_sfloat>, "invalid type");
    auto lhs_value = static_cast<ufloat_t>(*this);
    auto rhs_value = static_cast<ufloat_t>(other);
    return ch_sfloat(lhs_value + rhs_value);
  }

  template <typename R, typename U>
  auto do_sub(const U& other) const {
    static_assert(std::is_same_v<R, ch_sfloat>, "invalid type");
    auto lhs_value = static_cast<ufloat_t>(*this);
    auto rhs_value = static_cast<ufloat_t>(other);
    return ch_sfloat(lhs_value - rhs_value);
  }

  template <typename R, typename U>
  auto do_mul(const U& other) const {
    static_assert(std::is_same_v<R, ch_sfloat>, "invalid type");
    auto lhs_value = static_cast<ufloat_t>(*this);
    auto rhs_value = static_cast<ufloat_t>(other);
    return ch_sfloat(lhs_value * rhs_value);
  }

  template <typename R, typename U>
  auto do_div(const U& other) const {
    static_assert(std::is_same_v<R, ch_sfloat>, "invalid type");
    auto lhs_value = static_cast<ufloat_t>(*this);
    auto rhs_value = static_cast<ufloat_t>(other);
    return ch_sfloat(lhs_value / rhs_value);
  }

  template <typename R, typename U>
  auto do_mod(const U& other) const {
    static_assert(std::is_same_v<R, ch_sfloat>, "invalid type");
    auto lhs_value = static_cast<float>(*this);
    auto rhs_value = static_cast<float>(other);
    return ch_sfloat(fmod(lhs_value, rhs_value));
  }

  void do_print(std::ostream& out) const {
    out << static_cast<float>(*this);
  }

  const system_buffer& __buffer() const {
    return buffer_;
  }

  system_buffer buffer_;

  friend class ch::extension::system_accessor;
};

///////////////////////////////////////////////////////////////////////////////

template <unsigned M, unsigned E, unsigned TF=0>
class ch_float : public ch_numbase<ch_float<M, E, TF>> {
public:
  using ufloat_t = IEEE754<M, E, TF>;
  using uprimitive_t = typename ufloat_t::primitive;
  static constexpr unsigned N = ufloat_t::BITS;
  using traits = logic_traits<N, true, ch_float, ch_sfloat<M, E, TF>>;
  using base = ch_numbase<ch_float<M, E, TF>>;
  using base::operator=;

  explicit ch_float(const logic_buffer& buffer = make_logic_buffer(N, CH_CUR_SRC_INFO))
    : buffer_(buffer)
  {}

  explicit ch_float(const float& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }

  explicit ch_float(float&& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }

  explicit ch_float(const double& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }

  explicit ch_float(double&& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }

  template <typename U,
            CH_REQUIRES(ch_width_v<U> <= N)>
  explicit ch_float(const ch_sbitbase<U>& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    base::operator=(reinterpret_cast<const U&>(other));
  }

  template <typename U,
            CH_REQUIRES(ch_width_v<U> <= N)>
  explicit ch_float(const ch_bitbase<U>& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    base::operator=(reinterpret_cast<const U&>(other));
  }

  template <typename U,
            CH_REQUIRES(std::is_integral_v<U>)>
  ch_float(const U& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }
/*
  ch_float(const ch_sfloat& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }
*/
  ch_float(const ch_sfloat<M, E, TF>& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }

  ch_float(const ch_float& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }

  ch_float(const ufloat_t& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }

  ch_float(ufloat_t&& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }

  template <unsigned U,
            CH_REQUIRES(U == N)>
  ch_float(const ch_sbit<U>& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }

  template <unsigned U,
            CH_REQUIRES(U == N)>
  ch_float(const ch_bit<U>& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }

  template <unsigned U,
            CH_REQUIRES(U == N)>
  ch_float(const ch_suint<U>& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }

  template <unsigned U,
            CH_REQUIRES(U == N)>
  ch_float(const ch_uint<U>& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }

  template <unsigned U,
            CH_REQUIRES(U == N)>
  ch_float(const ch_sint<U>& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }

  template <unsigned U,
            CH_REQUIRES(U == N)>
  ch_float(const ch_int<U>& other, CH_SRC_INFO)
    : ch_float(make_logic_buffer(N, srcinfo)) {
    this->operator=(other);
  }

  ch_float(ch_float&& other) : buffer_(std::move(other.buffer_)) {}

  ch_float& operator=(const ch_float& other) {
    logic_accessor::assign(*this, other);
    return *this;
  }

  ch_float& operator=(ch_float&& other) {
    logic_accessor::move(*this, std::move(other));
    return *this;
  }

  ch_float& operator=(const ufloat_t& other) {
    *this = other.data;
    return *this;
  }

  ch_float& operator=(ufloat_t&& other) {
    *this = std::move(other.data);
    return *this;
  }

  ch_float& operator=(const float& other) {
    static_assert(M == 23, "invalid ufloat size");
    static_assert(E == 8, "invalid ufloat size");
    ufloat_t ufloat = ufloat_t(other);
    *this = ufloat.data;
    return *this;
  }

  ch_float& operator=(const double& other) {
    static_assert(M == 52, "invalid ufloat size");
    static_assert(E == 11, "invalid ufloat size");
    ufloat_t ufloat = ufloat_t(other);
    *this = ufloat.data;
    return *this;
  }

  ch_float& operator=(float&& other) {
    static_assert(M == 23, "invalid ufloat size");
    static_assert(E == 8, "invalid ufloat size");
    ufloat_t ufloat = ufloat_t(other);
    *this = ufloat.data;
    return *this;
  }

  ch_float& operator=(double&& other) {
    static_assert(M == 52, "invalid ufloat size");
    static_assert(E == 11, "invalid ufloat size");
    ufloat_t ufloat = ufloat_t(other);
    *this = ufloat.data;
    return *this;
  }

  ch_bit<M> mant() const {
    return ch_sliceref<E>(this->as_bit(), 0);
  }

  ch_bit<E> exp() const {
    return ch_sliceref<E>(this->as_bit(), M);
  }

  ch_bit<1> sign() const {
    return ch_sliceref<1>(this->as_bit(), M + E);
  }


protected:

  template <typename R>
  auto do_neg() const;

  template <typename R, typename U>
  auto do_add(const U& other) const;

  template <typename R, typename U>
  auto do_sub(const U& other) const;

  template <typename R, typename U>
  auto do_mul(const U& other) const;

  template <typename R, typename U>
  auto do_div(const U& other) const;

  template <typename R, typename U>
  auto do_mod(const U& other) const;

  void do_print(ch_ostream& out) const {
    out.write(get_lnode(*this), 'f');
  }

  const logic_buffer& __buffer() const {
    return buffer_;
  }

  logic_buffer buffer_;

  friend class ch::extension::logic_accessor;
};

///////////////////////////////////////////////////////////////////////////////

namespace detail {
template <typename T>
class sc_pipereg {
public:
  sc_pipereg(unsigned depth) : buffer_(depth), index_(0) {}

  T eval(const T& obj, bool enable) {
    if (enable) {
      buffer_[index_] = obj;
      if (++index_ == buffer_.size())
        index_ = 0;
    }
    return buffer_[index_];
  }

  void reset() {
    index_ = 0;
  }

private:
  std::vector<T> buffer_;
  unsigned index_;
};
}

template <unsigned M, unsigned E, unsigned TF=0>
class _ufAdd {
public:
  using ufloat_t = IEEE754<M, E, TF>;
  __sio (
    __in (ch_bool)   en,
    __in (ch_float<M, E, TF>)  lhs,
    __in (ch_float<M, E, TF>)  rhs,
    __out (ch_float<M, E, TF>) dst
  );

  _ufAdd(unsigned delay) : pipe_(delay) {}

  void eval() {
    auto lhs = static_cast<ufloat_t>(io.lhs);
    auto rhs = static_cast<ufloat_t>(io.rhs);
    io.dst = pipe_.eval(lhs + rhs, !!io.en);
  }

  void reset() {
    pipe_.reset();
  }

  bool to_verilog(udf_vostream& out, udf_verilog mode) {
    if (mode != udf_verilog::body)
      return false;
    out << "fp_add __fp_add$id(.clock($clk), .clk_en($io.en), "
           ".dataa($io.lhs), .datab($io.rhs), .result($io.dst));";
    return true;
  }

private:
  detail::sc_pipereg<ufloat_t> pipe_;
};

template <unsigned M, unsigned E, unsigned TF=0>
class _ufSub {
public:
  using ufloat_t = IEEE754<M, E, TF>;
  __sio (
    __in (ch_bool)     en,
    __in (ch_float<M, E, TF>)  lhs,
    __in (ch_float<M, E, TF>)  rhs,
    __out (ch_float<M, E, TF>) dst
  );

  _ufSub(unsigned delay) : pipe_(delay) {}

  void eval() {
    auto lhs = static_cast<ufloat_t>(io.lhs);
    auto rhs = static_cast<ufloat_t>(io.rhs);
    io.dst = pipe_.eval(lhs - rhs, !!io.en);
  }

  void reset() {
    pipe_.reset();
  }

  bool to_verilog(udf_vostream& out, udf_verilog mode) {
    if (mode != udf_verilog::body)
      return false;
    out << "fp_sub __fp_sub$id(.clock($clk), .clk_en($io.en), "
           ".dataa($io.lhs), .datab($io.rhs), .result($io.dst));";
    return true;
  }

private:
  detail::sc_pipereg<ufloat_t> pipe_;
};

template <unsigned M, unsigned E, unsigned TF=0>
class _ufMul {
public:
  using ufloat_t = IEEE754<M, E, TF>;
  __sio (
    __in (ch_bool)     en,
    __in (ch_float<M, E, TF>)  lhs,
    __in (ch_float<M, E, TF>)  rhs,
    __out (ch_float<M, E, TF>) dst
  );

  _ufMul(unsigned delay) : pipe_(delay) {}

  void eval() {
    auto lhs = static_cast<ufloat_t>(io.lhs);
    auto rhs = static_cast<ufloat_t>(io.rhs);
    io.dst = pipe_.eval(lhs * rhs, !!io.en);
  }

  void reset() {
    pipe_.reset();
  }

  bool to_verilog(udf_vostream& out, udf_verilog mode) {
    if (mode != udf_verilog::body)
      return false;
    out << "fp_mul __fp_mul$id("
        << ".clock($clk), "
        << ".clk_en(" << io.en << "), "
        << ".dataa(" << io.lhs << "), "
        << ".datab(" << io.rhs << "), "
        << ".result(" << io.dst << "));";
    return true;
  }

private:
  detail::sc_pipereg<ufloat_t> pipe_;
};

template <unsigned M, unsigned E, unsigned TF=0>
class _ufDiv {
public:
  using ufloat_t = IEEE754<M, E, TF>;
  __sio (
    __in (ch_bool)     en,
    __in (ch_float<M, E, TF>)  lhs,
    __in (ch_float<M, E, TF>)  rhs,
    __out (ch_float<M, E, TF>) dst
  );

  _ufDiv(unsigned delay) : pipe_(delay) {}

  void eval() {
    auto lhs = static_cast<ufloat_t>(io.lhs);
    auto rhs = static_cast<ufloat_t>(io.rhs);
    io.dst = pipe_.eval(lhs / rhs, !!io.en);
  }

  void reset() {
    pipe_.reset();
  }

  bool to_verilog(udf_vostream& out, udf_verilog mode) {
    if (mode != udf_verilog::body)
      return false;
    out << "fp_div __fp_div$id("
        << ".clock($clk), "
        << ".clk_en(" << io.en << "), "
        << ".dataa(" << io.lhs << "), "
        << ".datab(" << io.rhs << "), "
        << ".result(" << io.dst << "));";
    return true;
  }

private:
  detail::sc_pipereg<ufloat_t> pipe_;
};

template <unsigned M, unsigned E, unsigned TF=0>
class _ufSfu {
public:
  using ufloat_t = IEEE754<M, E, TF>;
  __sio (
    __in (ch_bool)     en,
    __in (ch_float<M, E, TF>)  lhs,
    __in (ch_float<M, E, TF>)  rhs,
    __out (ch_float<M, E, TF>) dst
  );

  _ufSfu(unsigned delay) : pipe_(delay) {}

  void eval() {
    auto lhs = static_cast<ufloat_t>(io.lhs);
    auto rhs = static_cast<ufloat_t>(io.rhs);
    io.dst = pipe_.eval(lhs * rhs, !!io.en);
  }

  void reset() {
    pipe_.reset();
  }

  bool to_verilog(udf_vostream& out, udf_verilog mode) {
    return false;
  }

private:
  detail::sc_pipereg<ufloat_t> pipe_;
};

///////////////////////////////////////////////////////////////////////////////
template <unsigned M, unsigned E, unsigned TF=0>
struct cfAdd {
  using ufloat_t = IEEE754<M, E, TF>;
  __sio (
    __in (ch_float<M, E, TF>)  lhs,
    __in (ch_float<M, E, TF>)  rhs,
    __out (ch_float<M, E, TF>) dst
  );

  void eval() {
    auto lhs = static_cast<ufloat_t>(io.lhs);
    auto rhs = static_cast<ufloat_t>(io.rhs);
    //auto lhs = bit_cast<float>(static_cast<int32_t>(io.lhs));
    //auto rhs = bit_cast<float>(static_cast<int32_t>(io.rhs));
    // nio.dst = bit_cast<int32_t>(lhs + rhs);
    io.dst = lhs + rhs;
  }
};

template <unsigned M, unsigned E, unsigned TF=0>
struct cfSub {
  using ufloat_t = IEEE754<M, E, TF>;
  __sio (
    __in (ch_float<M, E, TF>)  lhs,
    __in (ch_float<M, E, TF>)  rhs,
    __out (ch_float<M, E, TF>) dst
  );

  void eval() {
    auto lhs = static_cast<ufloat_t>(io.lhs);
    auto rhs = static_cast<ufloat_t>(io.rhs);
    // auto lhs = bit_cast<float>(static_cast<int32_t>(io.lhs));
    // auto rhs = bit_cast<float>(static_cast<int32_t>(io.rhs));
    //io.dst = bit_cast<int32_t>(lhs - rhs);
    io.dst = lhs - rhs;
  }
};

template <unsigned M, unsigned E, unsigned TF=0>
struct cfMul {
  using ufloat_t = IEEE754<M, E, TF>;
  __sio (
    __in (ch_float<M, E, TF>)  lhs,
    __in (ch_float<M, E, TF>)  rhs,
    __out (ch_float<M, E, TF>) dst
  );

  void eval() {
    auto lhs = static_cast<ufloat_t>(io.lhs);
    auto rhs = static_cast<ufloat_t>(io.rhs);
    io.dst = lhs * rhs;
    // auto lhs = bit_cast<float>(static_cast<int32_t>(io.lhs));
    // auto rhs = bit_cast<float>(static_cast<int32_t>(io.rhs));
    // io.dst = bit_cast<int32_t>(lhs * rhs);
  }
};

template <unsigned M, unsigned E, unsigned TF=0>
struct cfDiv {
  using ufloat_t = IEEE754<M, E, TF>;
  __sio (
    __in (ch_float<M, E, TF>)  lhs,
    __in (ch_float<M, E, TF>)  rhs,
    __out (ch_float<M, E, TF>) dst
  );

  void eval() {
    auto lhs = static_cast<ufloat_t>(io.lhs);
    auto rhs = static_cast<ufloat_t>(io.rhs);
    io.dst = lhs / rhs;
    // auto lhs = bit_cast<float>(static_cast<int32_t>(io.lhs));
    // auto rhs = bit_cast<float>(static_cast<int32_t>(io.rhs));
    // io.dst = bit_cast<int32_t>(lhs / rhs);
  }
};
#if  0
template <unsigned M, unsigned E>
struct cfMod {
  __sio (
    __in (ch_float<M, E>)  lhs,
    __in (ch_float<M, E>)  rhs,
    __out (ch_float<M, E>) dst
  );

  void eval() {
    auto lhs = bit_cast<float>(static_cast<int32_t>(io.lhs));
    auto rhs = bit_cast<float>(static_cast<int32_t>(io.rhs));
    io.dst = bit_cast<int32_t>(fmod(lhs, rhs));
  }
};
#endif
///////////////////////////////////////////////////////////////////////////////

template <unsigned M, unsigned E, unsigned TF>
template <typename R>
auto ch_float<M, E, TF>::do_neg() const {
  static_assert(std::is_same_v<R, ch_float<M, E, TF>>, "invalid type");
  ch_udf_comb<cfSub<M,E,TF>> udf;
  udf.io.lhs = 0.0f;
  udf.io.rhs = *this;
  return udf.io.dst;
}

template <unsigned M, unsigned E, unsigned TF>
template <typename R, typename U>
auto ch_float<M, E, TF>::do_add(const U& other) const {
  static_assert(std::is_same_v<R, ch_float<M, E, TF>>, "invalid type");
  ch_udf_comb<cfAdd<M, E, TF>> udf;
  udf.io.lhs = *this;
  udf.io.rhs = other;
  return udf.io.dst;
}

template <unsigned M, unsigned E, unsigned TF>
template <typename R, typename U>
auto ch_float<M, E, TF>::do_sub(const U& other) const {
  static_assert(std::is_same_v<R, ch_float<M, E, TF>>, "invalid type");
  ch_udf_comb<cfSub<M, E, TF>> udf;
  udf.io.lhs = *this;
  udf.io.rhs = other;
  return udf.io.dst;
}

template <unsigned M, unsigned E, unsigned TF>
template <typename R, typename U>
auto ch_float<M, E, TF>::do_mul(const U& other) const {
  static_assert(std::is_same_v<R, ch_float<M, E, TF>>, "invalid type");
  ch_udf_comb<cfMul<M, E, TF>> udf;
  udf.io.lhs = *this;
  udf.io.rhs = other;
  return udf.io.dst;
}

template <unsigned M, unsigned E, unsigned TF>
template <typename R, typename U>
auto ch_float<M, E, TF>::do_div(const U& other) const {
  static_assert(std::is_same_v<R, ch_float<M, E, TF>>, "invalid type");
  ch_udf_comb<cfDiv<M, E, TF>> udf;
  udf.io.lhs = *this;
  udf.io.rhs = other;
  return udf.io.dst;
}

#if 0
template <unsigned M, unsigned E>
template <typename R, typename U>
auto ch_float<M, E>::do_mod(const U& other) const {
  static_assert(std::is_same_v<R, ch_float<M, E>>, "invalid type");
  ch_udf_comb<cfMod<M, E>> udf;
  udf.io.lhs = *this;
  udf.io.rhs = other;
  return udf.io.dst;
}
#endif

///////////////////////////////////////////////////////////////////////////////
template <unsigned M, unsigned E, unsigned TF=0, unsigned Delay = 1>
auto ufadd(const ch_float<M, E, TF>& lhs, const ch_float<M, E, TF>& rhs, const ch_bool& enable = true, CH_SRC_INFO) {
  ch_udf_seq<_ufAdd<M, E, TF>> udf(Delay, srcinfo);
  udf.io.en  = enable;
  udf.io.lhs = lhs;
  udf.io.rhs = rhs;
  return udf.io.dst;
}

template <unsigned M, unsigned E, unsigned TF=0, unsigned Delay = 1>
auto ufsub(const ch_float<M, E, TF>& lhs, const ch_float<M, E, TF>& rhs, const ch_bool& enable = true, CH_SRC_INFO) {
  ch_udf_seq<_ufSub<M, E, TF>> udf(Delay, srcinfo);
  udf.io.en  = enable;
  udf.io.lhs = lhs;
  udf.io.rhs = rhs;
  return udf.io.dst;
}

template <unsigned M, unsigned E, unsigned TF=0, unsigned Delay = 1>
auto ufmul(const ch_float<M, E, TF>& lhs, const ch_float<M, E, TF>& rhs, const ch_bool& enable = true, CH_SRC_INFO) {
  ch_udf_seq<_ufMul<M, E, TF>> udf(Delay, srcinfo);
  udf.io.en  = enable;
  udf.io.lhs = lhs;
  udf.io.rhs = rhs;
  return udf.io.dst;
}

template <unsigned M, unsigned E, unsigned TF=0, unsigned Delay = 1>
auto ufdiv(const ch_float<M, E, TF>& lhs, const ch_float<M, E, TF>& rhs, const ch_bool& enable = true, CH_SRC_INFO) {
  ch_udf_seq<_ufDiv<M, E, TF>> udf(Delay, srcinfo);
  udf.io.en  = enable;
  udf.io.lhs = lhs;
  udf.io.rhs = rhs;
  return udf.io.dst;
}

}
}
