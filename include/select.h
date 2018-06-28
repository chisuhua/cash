#pragma once

#include "bit.h"

namespace ch {
namespace internal {

class select_impl {
public:
  
  select_impl(const source_location& sloc)
    : sloc_(sloc)
  {}

  select_impl(const lnode& key, const source_location& sloc)
    : key_(key)
    , sloc_(sloc)
  {}

  ~select_impl() {}
  
  void push(const lnode& pred, const lnode& value) {
    stmts_.push({pred, value});
  }
  
  lnodeimpl* eval(const lnode& value);

  auto sloc() const {
    return sloc_;
  }

protected:

  struct stmt_t {
    lnode pred;
    lnode value;
  };

  lnode key_;
  std::stack<stmt_t> stmts_;
  source_location sloc_;
};

template <typename T>
class select_t {
public:
    
  select_t(const lnode& pred,
           const lnode& value,
           const source_location& sloc)
    : impl_(sloc) {
    impl_.push(pred, value);
  }

  template <typename V>
  select_t<T>& operator()(const ch_bit<1>& pred, const V& value) {
    static_assert(std::is_constructible_v<T, V>, "invalid type");
    impl_.push(get_lnode(pred), to_lnode<T>(value, impl_.sloc()));
    return *this;
  }
  
  template <typename V>
  auto operator()(const V& value) {
    static_assert(std::is_constructible_v<T, V>, "invalid type");
    return make_type<T>(impl_.eval(to_lnode<T>(value, impl_.sloc())), impl_.sloc());
  }
  
protected:
  
  select_impl impl_;
};

template <typename K, typename V>
class case_t {
public:

  case_t(const lnode& key,
         const lnode& pred,
         const lnode& value,
         const source_location& sloc)
    : impl_(key, sloc) {
    impl_.push(pred, value);
  }

  template <typename P, typename T>
  case_t& operator()(const P& pred, const T& value) {
    static_assert(is_equality_comparable_v<P, K>, "invalid type");
    static_assert(std::is_constructible_v<V, T>, "invalid type");
    impl_.push(to_lnode<ch_width_v<K>>(pred, impl_.sloc()),
               to_lnode<V>(value, impl_.sloc()));
    return *this;
  }

  template <typename T>
  auto operator()(const T& value) {
    static_assert(std::is_constructible_v<V, T>, "invalid type");
    return make_type<V>(impl_.eval(to_lnode<V>(value, impl_.sloc())),
                        impl_.sloc());
  }
  
protected:
  
  select_impl impl_;
};

template <typename R, typename T>
auto ch_sel(const ch_bit<1>& pred, const T& value, CH_SLOC) {
  static_assert(is_logic_type_v<R>, "invalid type");
  static_assert(std::is_constructible_v<R, T>, "invalid type");
  return select_t<R>(get_lnode(pred), to_lnode<R>(value, sloc), sloc);
}

template <typename T>
auto ch_sel(const ch_bit<1>& pred, const T& value, CH_SLOC) {
  static_assert(is_object_type_v<T>, "invalid type");
  return select_t<ch_logic_t<T>>(get_lnode(pred), to_lnode<ch_width_v<T>>(value, sloc), sloc);
}

template <typename R, typename U, typename V>
auto ch_sel(const ch_bit<1>& pred, const U& _true, const V& _false, CH_SLOC) {
  return ch_sel<R, U>(pred, _true, sloc)(_false);
}

template <typename U, typename V>
auto ch_sel(const ch_bit<1>& pred, const U& _true, const V& _false, CH_SLOC) {  
  static_assert(ch_width_v<deduce_type_t<false, U, V>> != 0, "invalid type");
  return ch_sel<ch_logic_t<deduce_first_type_t<U, V>>, U, V>(pred, _true, _false, sloc);
}

template <typename U, typename V>
auto ch_min(const U& lhs, const V& rhs, CH_SLOC) {
  return ch_sel(lhs < rhs, lhs, rhs, sloc);
}

template <typename U, typename V>
auto ch_max(const U& lhs, const V& rhs, CH_SLOC) {
  return ch_sel(lhs > rhs, lhs, rhs, sloc);
}

template <typename R, typename K, typename V, typename P>
auto ch_case(const K& key, const P& pred, const V& value, CH_SLOC) {
  static_assert(is_logic_type_v<K>, "invalid type");
  static_assert(is_scbit_convertible_v<P, ch_width_v<K>>, "invalid type");
  static_assert(is_equality_comparable_v<P, K>, "invalid type");
  static_assert(std::is_constructible_v<R, V>, "invalid type");
  return case_t<K, R>(get_lnode(key),
                      to_lnode<ch_width_v<K>>(pred, sloc),
                      to_lnode<R>(value, sloc),
                      sloc);
}

template <typename K, typename V, typename P>
auto ch_case(const K& key, const P& pred, const V& value, CH_SLOC) {
  static_assert(is_object_type_v<V>, "invalid type");
  return ch_case<ch_logic_t<V>, K, V, P>(key, pred, value, sloc);
}

}
}
