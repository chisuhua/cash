#pragma once

#include "bitbase.h"

namespace cash {
namespace internal {

void createPrintNode(const std::string& format,
                     const std::initializer_list<lnode>& args);

template <unsigned N> class ch_bus;

#define CH_BIT_READONLY_INTERFACE(type) \
  const auto operator[](size_t index) const & { \
    return cash::internal::const_sliceref<cash::internal::refproxy<type>, 1>(*this, index); \
  } \
  const auto operator[](size_t index) const && { \
    return cash::internal::const_sliceref<type, 1>(std::move(*this), index); \
  } \
  template <unsigned M> \
  const auto slice(size_t index = 0) const & { \
    return cash::internal::const_sliceref<cash::internal::refproxy<type>, M>(*this, index); \
  } \
  template <unsigned M> \
  const auto slice(size_t index = 0) const && { \
    return cash::internal::const_sliceref<type, M>(std::move(*this), index); \
  } \
  template <unsigned M> \
  const auto aslice(size_t index = 0) const & { \
    return cash::internal::const_sliceref<cash::internal::refproxy<type>, M>(*this, index * M); \
  } \
  template <unsigned M> \
  const auto aslice(size_t index = 0) const && { \
    return cash::internal::const_sliceref<type, M>(std::move(*this), index * M); \
  }

#define CH_BIT_WRITABLE_INTERFACE(type) \
  type& operator=(const ch_bitbase<type::bitcount>& rhs) { \
    this->assign(rhs); \
    return *this; \
  } \
  template <typename U, CH_REQUIRES(cash::internal::is_bit_scalar<U>::value)> \
  type& operator=(U rhs) { \
    this->assign(rhs); \
    return *this; \
  } \
  auto operator[](size_t index) & { \
    return cash::internal::sliceref<type, 1>(*this, index); \
  } \
  const auto operator[](size_t index) const & { \
    return cash::internal::const_sliceref<cash::internal::refproxy<type>, 1>(*this, index); \
  } \
  const auto operator[](size_t index) const && { \
    return cash::internal::const_sliceref<type, 1>(std::move(*this), index); \
  } \
  template <unsigned M> \
  auto slice(size_t index = 0) & { \
    return cash::internal::sliceref<type, M>(*this, index); \
  } \
  template <unsigned M> \
  const auto slice(size_t index = 0) const & { \
    return cash::internal::const_sliceref<cash::internal::refproxy<type>, M>(*this, index); \
  } \
  template <unsigned M> \
  const auto slice(size_t index = 0) const && { \
    return cash::internal::const_sliceref<type, M>(std::move(*this), index); \
  } \
  template <unsigned M> \
  auto aslice(size_t index = 0) & { \
    return cash::internal::sliceref<type, M>(*this, index * M); \
  } \
  template <unsigned M> \
  const auto aslice(size_t index = 0) const & { \
    return cash::internal::const_sliceref<cash::internal::refproxy<type>, M>(*this, index * M); \
  } \
  template <unsigned M> \
  const auto aslice(size_t index = 0) const && { \
    return cash::internal::const_sliceref<type, M>(std::move(*this), index * M); \
  }

template <unsigned N>
class const_bit : public ch_bitbase<N> {
public:
  using base = ch_bitbase<N>;
  using data_type  = lnode;
  using value_type = ch_bit<N>;
  using const_type = const_bit<N>;
  using bus_type   = ch_bus<N>;

  const_bit() : node_(N) {}

  const_bit(const const_bit& rhs) : node_(rhs.node_, N) {}

  const_bit(const_bit&& rhs) : node_(std::move(rhs.node_), N) {}

  const_bit(const ch_bitbase<N>& rhs) : node_(get_lnode(rhs), N) {}

  template <typename U,
            CH_REQUIRES(is_bit_scalar<U>::value)>
  explicit const_bit(U rhs) : node_(bitvector(N, rhs)) {}

  const auto clone() const {
    return make_bit<N>(node_.clone(N));
  }

  CH_BIT_READONLY_INTERFACE(const_bit)

protected:

  const_bit(const lnode& node) : node_(node) {
    assert(N == node_.get_size());
  }

  void read_data(nodelist<lnode>& inout, size_t offset, size_t length) const override {
    node_.read_data(inout, offset, length, N);
  }

  void write_data(size_t dst_offset, const nodelist<lnode>& in, size_t src_offset, size_t length) override {
    node_.write_data(dst_offset, in, src_offset, length, N);
  }

  lnode node_;
};

template <unsigned N>
class ch_bit : public const_bit<N> {
public:
  using base = const_bit<N>;
  using data_type  = lnode;
  using value_type = ch_bit<N>;
  using const_type = const_bit<N>;
  using bus_type   = ch_bus<N>;

  using base::node_;
      
  ch_bit() {}
  
  ch_bit(const ch_bit& rhs) : base(rhs) {}

  ch_bit(const const_bit<N>& rhs) : base(rhs) {}

  ch_bit(ch_bit&& rhs) : base(rhs) {}

  ch_bit(const ch_bitbase<N>& rhs) : base(rhs) {}
    
  template <typename U,
            CH_REQUIRES(is_bit_scalar<U>::value)>
  explicit ch_bit(U rhs) : base(rhs) {}

  ch_bit& operator=(const ch_bit& rhs) {
    node_.assign(rhs.node_, N);
    return *this;
  }

  ch_bit& operator=(ch_bit&& rhs) {
    node_.move(rhs.node_, N);
    return *this;
  }

  CH_BIT_WRITABLE_INTERFACE(ch_bit)
  
protected:

  ch_bit(const lnode& node) : base(node) {}

  template <unsigned M> friend const ch_bit<M> make_bit(const lnode& node);
};

template <unsigned N>
const ch_bit<N> make_bit(const lnode& node) {
  return ch_bit<N>(node);
}

template <typename T, unsigned N = std::decay<T>::type::bitcount>
struct is_bit_convertible {
  static constexpr bool value = is_cast_convertible<T, ch_bit<N>>::value;
};

template <typename... Ts>
struct are_bit_convertible;

template <typename T>
struct are_bit_convertible<T> {
  static constexpr bool value = is_cast_convertible<T, ch_bit<std::decay<T>::type::bitcount>>::value;
};

template <typename T0, typename... Ts>
struct are_bit_convertible<T0, Ts...> {
  static constexpr bool value = is_cast_convertible<T0, ch_bit<std::decay<T0>::type::bitcount>>::value && are_bit_convertible<Ts...>::value;
};

template <typename T, unsigned N = T::bitcount>
using bit_cast = std::conditional<
  std::is_base_of<ch_bitbase<N>, T>::value,
  const ch_bitbase<N>&,
  ch_bit<N>>;

template <typename T, unsigned N = T::bitcount,
          CH_REQUIRES(is_bit_convertible<T, N>::value)>
lnode get_lnode(const T& rhs) {
  nodelist<lnode> data(N);
  typename bit_cast<T, N>::type x(rhs);
  read_data(x, data, 0, N);
  return lnode(data);
}

// concatenation functions

template <typename T>
void ch_cat_helper(nodelist<lnode>& data, const T& arg) {
  read_data(arg, data, 0, T::bitcount);
}

template <typename T0, typename... Ts>
void ch_cat_helper(nodelist<lnode>& data, const T0& arg0, const Ts&... args) {
  ch_cat_helper(data, args...);
  read_data(arg0, data, 0, T0::bitcount);
}

template <typename... Ts>
const auto cat_impl(const Ts&... args) {
  nodelist<lnode> data(concat_info<Ts...>::bitcount);
  ch_cat_helper(data, args...);
  return make_bit<concat_info<Ts...>::bitcount>(lnode(data));
}

template <typename... Ts,
         CH_REQUIRES(are_bit_convertible<Ts...>::value)>
const auto ch_cat(const Ts&... args) {
  return cat_impl(static_cast<typename bit_cast<Ts>::type>(args)...);
}

template <unsigned B, unsigned A>
const auto operator,(const ch_bitbase<B>& b, const ch_bitbase<A>& a) {
  return ch_cat(b, a);
}

template <typename... Ts,
         CH_REQUIRES(are_bit_convertible<Ts...>::value)>
concatref<Ts...> ch_tie(Ts&... args) {
  return concatref<Ts...>(args...);
}

// slice functions

template <unsigned N, typename T,
          CH_REQUIRES(is_bit_convertible<T>::value)>
const auto ch_slice(const T&& in, size_t index = 0) {
  return std::move(in).template slice<N>(index);
}

template <unsigned N, typename T,
          CH_REQUIRES(is_bit_convertible<T>::value)>
const auto ch_aslice(const T&& in, size_t index = 0) {
  return std::move(in).template aslice<N>(index);
}

template <unsigned N, unsigned M>
const auto ch_slice(const ch_bitbase<M>& in, size_t index = 0) {
  return in.template slice<N>(index);
}

template <unsigned N, unsigned M>
const auto ch_aslice(const ch_bitbase<M>& in, size_t index = 0) {
  return in.template aslice<N>(index);
}

// extend functions

template <unsigned D>
class zext_select {
public:
    template <unsigned N>
    const auto operator() (const ch_bitbase<N>& in) {
      return ch_cat(ch_bit<D>(0x0), in);
    }
};

template <>
class zext_select<0> {
public:
    template <unsigned N>
    const auto operator() (const ch_bitbase<N>& in) {
      return ch_bit<N>(in);
    }
};

template <unsigned M, unsigned D>
class sext_pad {
public:
  template <unsigned N>
  const auto operator() (const ch_bitbase<N>& in) {
    return ch_cat(in[M-1], ch_bit<D>(0x0), ch_slice<M-1>(in, 1));
  }
};

template <unsigned D>
class sext_pad<1, D> {
public:
  template <unsigned N>
  const auto operator() (const ch_bitbase<N>& in) {
    return ch_cat(in, ch_bit<D>(0x0));
  }
};

template <unsigned D>
class sext_select {
public:
  template <unsigned N>
  const auto operator() (const ch_bitbase<N>& in) {
    return sext_pad<N, D>()(in);
  }
};

template <>
class sext_select<0> {
public:
  template <unsigned N>
  const auto operator() (const ch_bitbase<N>& in) {
    return ch_bit<N>(in);
  }
};

template <unsigned N, unsigned M>
const auto ch_zext(const ch_bitbase<M>& in) {
  static_assert(N >= M, "invalid extend size");
  return zext_select<(N-M)>()(in);
}

template <unsigned N, unsigned M>
const auto ch_sext(const ch_bitbase<M>& in) {
  static_assert(N >= M, "invalid extend size");
  return sext_select<(N-M)>()(in);
}

// shuffle functions

template <unsigned N, unsigned I>
const auto ch_shuffle(const ch_bitbase<N>& in,
                     const std::array<uint32_t, I>& indices) {  
  static_assert((I % N) == 0, "invalid indices size");
  ch_bit<N> ret;
  for (unsigned i = 0; i < I; ++i) {
    ret.template aslice<<(N / I)>(i) = in.template aslice<(N / I)>(indices[i]);
  }
  return ret;
}

// print functions

const ch_bit<64> ch_getTick();

inline void ch_print(const std::string& format) {
  createPrintNode(format, {});
}

template <typename...Args,
          CH_REQUIRES(are_bit_convertible<Args...>::value)>
void ch_print(const std::string& format, const Args& ...args) {
  createPrintNode(format, {get_lnode(args)...});
}

// utils functions

template <typename... Ts>
struct return_type {
  using type = std::tuple<typename Ts::value_type...>;
};

template <typename U>
struct return_type<U> {
  using type = typename U::value_type;
};

template <typename T,
          CH_REQUIRES(is_bit_convertible<T>::value)>
const typename T::value_type make_return(const T& arg) {
  return arg;
}

template <typename T0, typename... Ts,
          CH_REQUIRES(is_bit_convertible<T0>::value),
          CH_REQUIRES(are_bit_convertible<Ts...>::value)>
const auto make_return(const T0& arg0, const Ts&... args) {
  return typename std::tuple<typename T0::value_type, typename Ts::value_type...>(arg0, args...);
}

}
}

#define CH_OUT(...) typename cash::internal::return_type<__VA_ARGS__>::type
#define CH_RET(...) return cash::internal::make_return(__VA_ARGS__)
#define CH_TIE(...) std::forward_as_tuple(__VA_ARGS__)
