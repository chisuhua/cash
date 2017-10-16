#pragma once

#include "bit.h"

namespace ch {
namespace internal {

lnodeimpl* createSelectNode(const lnode& pred, const lnode& a, const lnode& b);

class select_impl {
public:
  
  select_impl() {}

  select_impl(const lnode& key) : key_(key) {}

  ~select_impl() {}
  
  void push(const lnode& pred, const lnode& value) {
    stmts_.push({pred, value});
  }
  
  lnodeimpl* eval(const lnode& value);

protected:

  struct stmt_t {
    lnode pred;
    lnode value;
  };

  using stmts_t = std::stack<stmt_t>;

  lnode key_;
  stmts_t stmts_;
};

template <typename T>
class select_t {
public:
    
  select_t(const lnode& pred, const lnode& value) {
    impl_.push(pred, value);
  }

  template <typename V,
            CH_REQUIRES(is_cast_convertible<T, V>::value)>
  select_t<T>& operator()(const bitbase<1>& pred, const V& value) {
    impl_.push(get_lnode(pred), get_lnode<V, T::bitsize>(value));
    return *this;
  }
  
  template <typename V,
            CH_REQUIRES(is_cast_convertible<T, V>::value)>
  const auto operator()(const V& value) {
    return make_type<T>(impl_.eval(get_lnode<V, T::bitsize>(value)));
  }
  
protected:
  
  select_impl impl_;
};

template <typename K, typename V>
class case_t {
public:

  case_t(const lnode& key, const lnode& pred, const lnode& value) : impl_(key) {
    impl_.push(pred, value);
  }

  template <typename P, typename T,
            CH_REQUIRES(is_equality_comparable<P, K>::value),
            CH_REQUIRES(is_cast_convertible<V, T>::value)>
  case_t& operator()(const P& pred, const T& value) {
    impl_.push(get_lnode<P, K::bitsize>(pred), get_lnode<T, V::bitsize>(value));
    return *this;
  }

  template <typename T,
            CH_REQUIRES(is_cast_convertible<V, T>::value)>
  const auto operator()(const T& value) {
    return make_type<V>(impl_.eval(get_lnode<T, V::bitsize>(value)));
  }
  
protected:
  
  select_impl impl_;
};

template <typename V,
          CH_REQUIRES(is_ch_bit_convertible<V>::value)>
auto ch_select(const bitbase<1>& pred, const V& value) {
  return select_t<value_type_t<V>>(get_lnode(pred), get_lnode(value));
}

template <typename R, typename V,
          CH_REQUIRES(is_cast_convertible<R, V>::value)>
auto ch_select(const bitbase<1>& pred, const V& value) {
  return select_t<R>(
        get_lnode(pred), get_lnode<V, R::bitsize>(value));
}

template <typename K, typename P, typename V,
          CH_REQUIRES(is_ch_bit_convertible<K>::value),
          CH_REQUIRES(is_equality_comparable<P, K>::value),
          CH_REQUIRES(is_ch_bit_convertible<V>::value)>
auto ch_case(const K& key, const P& pred, const V& value) {
  return case_t<K, value_type_t<V>>(
        get_lnode(key), get_lnode<P, K::bitsize>(pred), get_lnode(value));
}

template <typename R, typename K, typename P, typename V,
          CH_REQUIRES(is_ch_bit_convertible<K>::value),
          CH_REQUIRES(is_equality_comparable<P, K>::value),
          CH_REQUIRES(is_cast_convertible<R, V>::value)>
auto ch_case(const K& key, const P& pred, const V& value) {
  return case_t<K, R>(
        get_lnode(key), get_lnode<P, K::bitsize>(pred), get_lnode<V, R::bitsize>(value));
}

template <typename U, typename V,
          CH_REQUIRES(deduce_type_t<U, V>::bitsize != 0),
          CH_REQUIRES(is_ch_bit_convertible<U, deduce_type_t<U, V>::bitsize>::value),
          CH_REQUIRES(is_ch_bit_convertible<V, deduce_type_t<U, V>::bitsize>::value)>
const auto ch_select(const bitbase<1>& pred, const U& _true, const V& _false) {
  return make_type<deduce_first_type_t<U, V>>(
        createSelectNode(get_lnode(pred),
                         get_lnode<U, deduce_type_t<U, V>::bitsize>(_true),
                         get_lnode<V, deduce_type_t<U, V>::bitsize>(_false)));
}

template <typename R, typename U, typename V,
          CH_REQUIRES(is_cast_convertible<R, U>::value),
          CH_REQUIRES(is_cast_convertible<R, V>::value)>
const auto ch_select(const bitbase<1>& pred, const U& _true, const V& _false) {
  return make_type<R>(
        createSelectNode(get_lnode(pred),
                         get_lnode<U, R::bitsize>(_true),
                         get_lnode<V, R::bitsize>(_false)));
}

template <typename U, typename V,
          CH_REQUIRES(deduce_type_t<U, V>::bitsize != 0),
          CH_REQUIRES(is_ch_bit_convertible<U, deduce_type_t<U, V>::bitsize>::value),
          CH_REQUIRES(is_ch_bit_convertible<V, deduce_type_t<U, V>::bitsize>::value)>
const auto ch_min(const U& lhs, const V& rhs) {
  return ch_select(lhs < rhs, lhs, rhs);
}

template <typename U, typename V,
          CH_REQUIRES(deduce_type_t<U, V>::bitsize != 0),
          CH_REQUIRES(is_ch_bit_convertible<U, deduce_type_t<U, V>::bitsize>::value),
          CH_REQUIRES(is_ch_bit_convertible<V, deduce_type_t<U, V>::bitsize>::value)>
const auto ch_max(const U& lhs, const V& rhs) {
  return ch_select(lhs > rhs, lhs, rhs);
}

}
}
