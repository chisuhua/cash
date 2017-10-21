#pragma once

#include "bit.h"

#define CH_UNION_SIZE_EACH(i, x) \
  ch::internal::identity_t<CH_PAIR_L(x)>::bitwidth

#define CH_UNION_SIZE(...) \
  std::max({CH_FOR_EACH(CH_UNION_SIZE_EACH, CH_SEP_COMMA, __VA_ARGS__)})

#define CH_UNION_SCALAR_DEFAULT_CTOR(i, x) \
  CH_PAIR_R(x)(ch::internal::scalar_buffer(ch::internal::identity_t<CH_PAIR_L(x)>::bitwidth, buffer, 0))

#define CH_UNION_DEFAULT_CTOR(i, x) \
  CH_PAIR_R(x)(ch::internal::bit_buffer(ch::internal::identity_t<CH_PAIR_L(x)>::bitwidth, buffer, 0))

#define CH_UNION_MOVE_CTOR(i, x) \
  CH_PAIR_R(x)(std::move(__rhs__.CH_PAIR_R(x)))

#define CH_UNION_MOVE_ASSIGN(i, x) \
  CH_PAIR_R(x) = std::move(__rhs__.CH_PAIR_R(x))

#define CH_UNION_SCALAR_GETBUFFER(i, x) \
  return ch::internal::scalar_accessor::get_buffer(CH_PAIR_R(x))

#define CH_UNION_GETBUFFER(i, x) \
  return ch::internal::bit_accessor::get_buffer(CH_PAIR_R(x))

#define CH_UNION_SCALAR_FIELD(i, x) \
  ch_scalar_t<ch::internal::identity_t<CH_PAIR_L(x)>> CH_PAIR_R(x)

#define CH_UNION_FIELD(i, x) \
  ch_value_t<ch::internal::identity_t<CH_PAIR_L(x)>> CH_PAIR_R(x)

#define CH_UNION_CONST_FIELD(i, x) \
  ch_const_t<ch::internal::identity_t<CH_PAIR_L(x)>> CH_PAIR_R(x)

#define CH_UNION_SCALAR_IMPL(union_name, value_name, ...) \
  class union_name { \
  public: \
    static constexpr unsigned bitwidth = CH_UNION_SIZE(__VA_ARGS__); \
    using traits = ch::internal::scalar_traits<union_name, value_name>; \
  public: \
    CH_FOR_EACH(CH_UNION_SCALAR_FIELD, CH_SEP_SEMICOLON, __VA_ARGS__); \
    union_name(const ch::internal::scalar_buffer& buffer = ch::internal::scalar_buffer(bitwidth)) \
      : CH_FOR_EACH(CH_UNION_SCALAR_DEFAULT_CTOR, CH_SEP_COMMA, __VA_ARGS__) { assert(bitwidth == buffer.get_size()); } \
    union_name(const union_name& __rhs__) \
      : union_name(ch::internal::scalar_accessor::cloneBuffer(__rhs__)) {} \
    union_name(union_name&& __rhs__) \
      : CH_FOR_EACH(CH_UNION_MOVE_CTOR, CH_SEP_COMMA, __VA_ARGS__) {} \
    template <typename __T__, CH_REQUIRES(ch::internal::is_bitvector_value<__T__>::value || std::is_enum<__T__>::value)> \
    explicit union_name(const __T__& __rhs__) \
      : union_name(ch::internal::scalar_buffer(ch::internal::bitvector(bitwidth, __rhs__))) {} \
    explicit union_name(const ch_scalar<bitwidth>& __rhs__) \
      : union_name(ch::internal::scalar_buffer(ch::internal::scalar_accessor::get_data(__rhs__))) {} \
    union_name& operator=(const union_name& __rhs__) { \
      ch::internal::scalar_accessor::copy(*this, __rhs__); \
      return *this; \
    } \
    union_name& operator=(union_name&& __rhs__) { \
      CH_FOR_EACH(CH_UNION_MOVE_ASSIGN, CH_SEP_SEMICOLON, __VA_ARGS__); \
      return *this; \
    } \
    CH_SCALAR_TYPE_INTERFACE(union_name) \
  private: \
    const ch::internal::scalar_buffer& get_buffer() const { \
      CH_FOR_EACH_1(0, CH_UNION_SCALAR_GETBUFFER, CH_SEP_SEMICOLON, __VA_ARGS__); \
    } \
    ch::internal::scalar_buffer& get_buffer() { \
      CH_FOR_EACH_1(0, CH_UNION_SCALAR_GETBUFFER, CH_SEP_SEMICOLON, __VA_ARGS__); \
    } \
    friend class ch::internal::scalar_accessor; \
  }

#define CH_UNION_BODY_IMPL(union_name, reverse_name, assignment_body, field_body, ...) \
  CH_FOR_EACH(field_body, CH_SEP_SEMICOLON, __VA_ARGS__); \
  union_name(const ch::internal::bit_buffer& buffer = ch::internal::bit_buffer(bitwidth)) \
    : CH_FOR_EACH(CH_UNION_DEFAULT_CTOR, CH_SEP_COMMA, __VA_ARGS__) { assert(bitwidth == buffer.get_size()); } \
  union_name(const union_name& __rhs__) \
    : union_name(ch::internal::bit_accessor::cloneBuffer(__rhs__)) {} \
  union_name(union_name&& __rhs__) \
    : CH_FOR_EACH(CH_UNION_MOVE_CTOR, CH_SEP_COMMA, __VA_ARGS__) {} \
  union_name(const reverse_name& __rhs__) \
    : union_name(ch::internal::bit_accessor::cloneBuffer(__rhs__)) {} \
  template <typename __T__, \
            CH_REQUIRES(ch::internal::is_bit_convertible<__T__, bitwidth>::value)> \
  explicit union_name(const __T__& __rhs__) \
    : union_name(ch::internal::bit_buffer(ch::internal::bit_accessor::get_data( \
                    static_cast<ch::internal::bit_cast_t<__T__, bitwidth>>(__rhs__)))) {} \
  assignment_body(union_name, __VA_ARGS__) \
protected: \
  const ch::internal::bit_buffer& get_buffer() const { \
    CH_FOR_EACH_1(0, CH_UNION_GETBUFFER, CH_SEP_SEMICOLON, __VA_ARGS__); \
  } \
  ch::internal::bit_buffer& get_buffer() { \
    CH_FOR_EACH_1(0, CH_UNION_GETBUFFER, CH_SEP_SEMICOLON, __VA_ARGS__); \
  } \
  friend class ch::internal::bit_accessor; \

#define CH_UNION_READONLY_IMPL(union_name, ...) \
  CH_BIT_READONLY_INTERFACE(union_name)

#define CH_UNION_WRITABLE_IMPL(union_name, ...) \
  CH_BIT_WRITABLE_INTERFACE(union_name) \
  union_name& operator=(const union_name& __rhs__) { \
    ch::internal::bit_accessor::copy(*this, __rhs__); \
    return *this; \
  } \
  union_name& operator=(union_name&& __rhs__) { \
    CH_FOR_EACH(CH_UNION_MOVE_ASSIGN, CH_SEP_SEMICOLON, __VA_ARGS__); \
    return *this; \
  } \
  union_name& operator=(const __const_type__& __rhs__) { \
    ch::internal::bit_accessor::copy(*this, __rhs__); \
    return *this; \
  }

#define CH_UNION_IMPL(union_name, ...) \
  class union_name { \
  protected: \
    CH_UNION_SCALAR_IMPL(__scalar_type__, union_name, __VA_ARGS__); \
    class __const_type__ { \
    public: \
      static constexpr unsigned bitwidth = CH_UNION_SIZE(__VA_ARGS__); \
      using traits = ch::internal::logic_traits<__const_type__, __const_type__, union_name, __scalar_type__>; \
      CH_UNION_BODY_IMPL(__const_type__, union_name, CH_UNION_READONLY_IMPL, CH_UNION_CONST_FIELD, __VA_ARGS__) \
    }; \
  public: \
    static constexpr unsigned bitwidth = CH_UNION_SIZE(__VA_ARGS__); \
    using traits = ch::internal::logic_traits<union_name, __const_type__, union_name, __scalar_type__>; \
    CH_UNION_BODY_IMPL(union_name, __const_type__, CH_UNION_WRITABLE_IMPL, CH_UNION_FIELD, __VA_ARGS__) \
  }

#define CH_UNION(name, body) \
  CH_UNION_IMPL(name, CH_REM body)
