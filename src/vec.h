#pragma once

#include "typebase.h"

namespace cash {
namespace internal {

template <typename T, unsigned N>
class ch_vec;

template <typename T, unsigned N>
class const_vec;

template <typename T, unsigned N>
class const_vec : public typebase<N * T::bitcount, typename T::data_type> {
public:
  using base = typebase<N * T::bitcount, typename T::data_type>;
  using data_type  = typename base::data_type;
  using value_type = ch_vec<T, N>;
  using const_type = const_vec;
  using bus_type   = ch_vec<typename data_traits<data_type>::template bus_type<T>, N>;
  
  const_vec() {}

  const_vec(const const_vec& rhs) : items_(rhs.items_) {}

  const_vec(const_vec&& rhs) : items_(std::move(rhs.items_)) {}

  template <typename U,
            CH_REQUIRES(is_cast_convertible<U, T>::value)>
  const_vec(const const_vec<U, N>& rhs) {
    for (unsigned i = 0; i < N; ++i) {
      items_[i] = rhs.items_[i];
    }
  }

  const_vec(const typebase<const_vec::bitcount, data_type>& rhs) {
    this->assign(rhs);
  }

  template <typename... Vs,
           CH_REQUIRES(are_all_cast_convertible<T, Vs...>::value)>
  explicit const_vec(const Vs&... values) {
    this->init(values...);
  }

  template <typename U,
            CH_REQUIRES(cash::internal::is_bit_scalar<U>::value)>
  explicit const_vec(U value) {
    this->assign(value);
  }

  const T& operator[](size_t i) const {
    CH_CHECK(i < N, "invalid subscript index");
    return items_[i];
  }

protected:
  
  std::array<T, N> items_;

  template <typename V>
  void init(const V& value) {
    items_[0] = value;
  }

  template <typename V0, typename... Vs>
  void init(const V0& value0, const Vs&... values) {
    items_[sizeof...(Vs)] = value0;
    this->init(values...);
  }
  
  void read_data(nodelist<data_type>& out, size_t offset, size_t length) const override {
    CH_CHECK(offset + length <= const_vec::bitcount, "invalid vector read range");
    for (unsigned i = 0; length && i < N; ++i) {
      if (offset < T::bitcount) {
        size_t len = std::min<size_t>(length, T::bitcount - offset);
        cash::internal::read_data(items_[i], out, offset, len);
        length -= len;
        offset = T::bitcount;
      }
      offset -= T::bitcount;
    }
  }
  
  void write_data(size_t dst_offset, const nodelist<data_type>& data, size_t src_offset, size_t length) override {
    CH_CHECK(dst_offset + length <= const_vec::bitcount, "invalid vector write range");
    for (unsigned i = 0; length && i < N; ++i) {
      if (dst_offset < T::bitcount) {
        size_t len = std::min<size_t>(length, T::bitcount - dst_offset);
        cash::internal::write_data(items_[i], dst_offset, data, src_offset, len);
        length -= len;
        src_offset += len;
        dst_offset = T::bitcount;
      }
      dst_offset -= T::bitcount;
    }
  }
};

template <typename T, unsigned N>
class ch_vec : public const_vec<T, N> {
public:
  using base = const_vec<T, N>;
  using data_type  = typename base::data_type;
  using value_type = ch_vec;
  using const_type = const_vec<T, N>;
  using bus_type   = ch_vec<typename data_traits<data_type>::template bus_type<T>, N>;

  using base::items_;

  ch_vec() {}

  ch_vec(const ch_vec& rhs) : base(rhs) {}

  ch_vec(ch_vec&& rhs) : base(rhs) {}

  template <typename U,
            CH_REQUIRES(is_cast_convertible<U, T>::value)>
  ch_vec(const const_vec<U, N>& rhs) : base(rhs) {}

  template <typename U,
            CH_REQUIRES(is_cast_convertible<U, T>::value)>
  ch_vec(const ch_vec<U, N>& rhs) : base(rhs) {}

  ch_vec(const typebase<ch_vec::bitcount, data_type>& rhs) : base(rhs) {}

  template <typename... Vs,
           CH_REQUIRES(are_all_cast_convertible<T, Vs...>::value)>
  explicit ch_vec(const Vs&... values) : base(values...) {}

  template <typename U,
            CH_REQUIRES(cash::internal::is_bit_scalar<U>::value)>
  explicit ch_vec(U value) : base(value) {} \

  ch_vec& operator=(const ch_vec& rhs) {
    items_ = rhs.items_;
    return *this;
  }

  ch_vec& operator=(ch_vec&& rhs) {
    items_ = std::move(rhs.items_);
    return *this;
  }

  template <typename U,
            CH_REQUIRES(is_cast_convertible<U, T>::value)>
  ch_vec& operator=(const ch_vec<U, N>& rhs) {
    for (unsigned i = 0; i < N; ++i) {
      items_[i] = rhs.items_[i];
    }
    return *this;
  }

  ch_vec& operator=(const typebase<ch_vec::bitcount, data_type>& rhs) {
    this->assign(rhs);
    return *this;
  }

  template <typename U, CH_REQUIRES(cash::internal::is_bit_scalar<U>::value)>
  ch_vec& operator=(U rhs) {
    this->assign(rhs);
    return *this;
  }

  const T& operator[](size_t i) const {
    CH_CHECK(i < N, "invalid subscript index");
    return items_[i];
  }

  T& operator[](size_t i) {
    CH_CHECK(i < N, "invalid subscript index");
    return items_[i];
  }
};

}
}
