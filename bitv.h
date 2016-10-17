#pragma once

#include "bitbase.h"
#include "vec.h"

#define CHDL_CONCAT_GEN(T, TcB, TcA, TB, TA, cB, cA, B, A) \
  T inline const_concat_ref<TcB, TcA> operator,(cB b, cA a) { return const_concat_ref<TcB, TcA>(b, a); } \
  T inline const_concat_ref<TcB, TA> operator,(cB b, A a) { return const_concat_ref<TcB, TA>(b, a); } \
  T inline const_concat_ref<TB, TcA> operator,(B b, cA a) { return const_concat_ref<TB, TcA>(b, a); } \
  T inline concat_ref<TB, TA> operator,(B b, A a) { return concat_ref<TB, TA>(b, a); } \
  T inline const_concat_ref<TcB, TcA> ch_concat(cB b, cA a) { return const_concat_ref<TcB, TcA>(b, a); } \
  T inline const_concat_ref<TcB, TA> ch_concat(cB b, A a) { return const_concat_ref<TcB, TA>(b, a); } \
  T inline const_concat_ref<TB, TcA> ch_concat(B b, cA a) { return const_concat_ref<TB, TcA>(b, a); } \
  T inline concat_ref<TB, TA> ch_concat(B b, A a) { return concat_ref<TB, TA>(b, a); }

namespace chdl_internal {

template <unsigned N> class ch_bus;
using ch_signal = ch_bus<1>;

template <unsigned N> class ch_bitv;
using ch_logic = ch_bitv<1>;

template <unsigned N> 
class ch_bitv : public ch_bitbase<N> {
public:
  using base = ch_bitbase<N>;
  typedef typename base::data_type data_type;
  typedef ch_bitv<N> logic_type;
  typedef ch_bus<N>  bus_type;
      
  ch_bitv() {}
  
  ch_bitv(const ch_bitv& rhs) : m_node(rhs.m_node, N) {}
  
  ch_bitv(const ch_bitbase<N>& rhs) {
    base::operator =(rhs);
  }
  
  ch_bitv(const std::string& value) : m_node(value) {
    assert(m_node.get_size() == N);
  }
  
  ch_bitv(const std::initializer_list<uint32_t>& value) : m_node(value, N) {}
  
  ch_bitv(uint32_t value) : m_node({value}, N) {}
  
  ch_bitv(char value) : m_node({to_value<N>(value)}, N) {} 
    
#define CHDL_DEF_CTOR(type) \
    ch_bitv(type value) : m_node({static_cast<uint32_t>(value)}, N) {}
  CHDL_DEF_CTOR(int8_t)
  CHDL_DEF_CTOR(uint8_t)
  CHDL_DEF_CTOR(int16_t)
  CHDL_DEF_CTOR(uint16_t)
  CHDL_DEF_CTOR(int32_t)
  CHDL_DEF_CTOR(int64_t)
  CHDL_DEF_CTOR(uint64_t)
#undef CHDL_DEF_CTOR
  
  explicit ch_bitv(const lnode& node) : m_node(node, N) {}
  
  ch_bitv& operator=(const ch_bitv& rhs) {
    rhs.m_node.ensureInitialized(0, N, N);
    m_node = rhs.m_node;
    return *this;
  }
  
  ch_bitv& operator=(const ch_bitbase<N>& rhs) {
    base::operator =(rhs);
    return *this;
  }
  
  ch_bitv& operator=(const std::initializer_list<uint32_t>& value) {
    m_node.assign(value, N);
    return *this;
  }

  ch_bitv& operator=(uint32_t value) {
    m_node.assign({value}, N);
    return *this;
  }

  ch_bitv& operator=(char value) {
    m_node.assign({to_value<N>(value)}, N);
    return *this;
  }
  
#define CHDL_DEF_AOP(type) \
  ch_bitv& operator=(type value) { \
    m_node.assign({static_cast<uint32_t>(value)}, N); \
    return *this; \
  } 
  CHDL_DEF_AOP(int8_t)
  CHDL_DEF_AOP(uint8_t)
  CHDL_DEF_AOP(int16_t)
  CHDL_DEF_AOP(uint16_t)
  CHDL_DEF_AOP(int32_t)
  CHDL_DEF_AOP(int64_t)
  CHDL_DEF_AOP(uint64_t)
#undef CHDL_DEF_AOP
  
  operator lnode() const { 
    m_node.ensureInitialized(0, N, N);
    return m_node; 
  }
  
protected:
  
  void read(std::vector< partition<data_type> >& out, size_t offset, size_t length) const override {
    m_node.read(out, offset, length, N);
  }
  
  void write(size_t dst_offset, const std::vector< partition<data_type> >& src, size_t src_offset, size_t src_length) override {
    m_node.write(dst_offset, src, src_offset, src_length, N);
  }
  
  lnode m_node;
};

// concatenation operator

CHDL_CONCAT_GEN(template <typename BB CHDL_COMMA typename BA CHDL_COMMA typename AB CHDL_COMMA typename AA>, 
                const_concat_ref<BB CHDL_COMMA BA>, const_concat_ref<AB CHDL_COMMA AA>,
                concat_ref<BB CHDL_COMMA BA>, concat_ref<AB CHDL_COMMA AA>,
                const const_concat_ref<BB CHDL_COMMA BA>&, const const_concat_ref<AB CHDL_COMMA AA>&,
                const concat_ref<BB CHDL_COMMA BA>&, const concat_ref<AB CHDL_COMMA AA>&)

CHDL_CONCAT_GEN(template <typename TB CHDL_COMMA unsigned NB CHDL_COMMA typename TA CHDL_COMMA unsigned NA>, 
                const_slice_ref<TB CHDL_COMMA NB>, const_slice_ref<TA CHDL_COMMA NA>,
                slice_ref<TB CHDL_COMMA NB>, slice_ref<TA CHDL_COMMA NA>,
                const const_slice_ref<TB CHDL_COMMA NB>&, const const_slice_ref<TA CHDL_COMMA NA>&,
                const slice_ref<TB CHDL_COMMA NB>&, const slice_ref<TA CHDL_COMMA NA>&)

CHDL_CONCAT_GEN(template <typename TB CHDL_COMMA typename TA>, 
                const_subscript_ref<TB>, const_subscript_ref<TA>,
                subscript_ref<TB>, subscript_ref<TA>,
                const const_subscript_ref<TB>&, const const_subscript_ref<TA>&,
                const subscript_ref<TB>&, const subscript_ref<TA>&)

CHDL_CONCAT_GEN(template <unsigned NB CHDL_COMMA unsigned NA>, 
                ch_bitbase<NB>, ch_bitbase<NA>,
                ch_bitbase<NB>, ch_bitbase<NA>,
                const ch_bitbase<NB>&, const ch_bitbase<NA>&,
                ch_bitbase<NB>&, ch_bitbase<NA>&)

CHDL_CONCAT_GEN(, 
                ch_logic, ch_logic,
                ch_logic, ch_logic,
                const ch_logic&, const ch_logic&,
                ch_logic&, ch_logic&)
//--

CHDL_CONCAT_GEN(template <typename BB CHDL_COMMA typename BA CHDL_COMMA typename TA CHDL_COMMA unsigned NA>, 
                const_concat_ref<BB CHDL_COMMA BA>, const_slice_ref<TA CHDL_COMMA NA>,
                concat_ref<BB CHDL_COMMA BA>, slice_ref<TA CHDL_COMMA NA>,
                const const_concat_ref<BB CHDL_COMMA BA>&, const const_slice_ref<TA CHDL_COMMA NA>&,
                const concat_ref<BB CHDL_COMMA BA>&, const slice_ref<TA CHDL_COMMA NA>&)

CHDL_CONCAT_GEN(template <typename BB CHDL_COMMA typename BA CHDL_COMMA typename TA>, 
                const_concat_ref<BB CHDL_COMMA BA>, const_subscript_ref<TA>,
                concat_ref<BB CHDL_COMMA BA>, subscript_ref<TA>,
                const const_concat_ref<BB CHDL_COMMA BA>&, const const_subscript_ref<TA>&,
                const concat_ref<BB CHDL_COMMA BA>&, const subscript_ref<TA>&)

CHDL_CONCAT_GEN(template <typename BB CHDL_COMMA typename BA CHDL_COMMA unsigned NA>, 
                const_concat_ref<BB CHDL_COMMA BA>, ch_bitbase<NA>,
                concat_ref<BB CHDL_COMMA BA>, ch_bitbase<NA>,
                const const_concat_ref<BB CHDL_COMMA BA>&, const ch_bitbase<NA>&,
                const concat_ref<BB CHDL_COMMA BA>&, ch_bitbase<NA>&)

CHDL_CONCAT_GEN(template <typename BB CHDL_COMMA typename BA>, 
                const_concat_ref<BB CHDL_COMMA BA>, ch_logic,
                concat_ref<BB CHDL_COMMA BA>, ch_logic,
                const const_concat_ref<BB CHDL_COMMA BA>&, const ch_logic&,
                const concat_ref<BB CHDL_COMMA BA>&, ch_logic&)

//--

CHDL_CONCAT_GEN(template <typename TB CHDL_COMMA unsigned NB CHDL_COMMA typename AB CHDL_COMMA typename AA>, 
                const_slice_ref<TB CHDL_COMMA NB>, const_concat_ref<AB CHDL_COMMA AA>,
                slice_ref<TB CHDL_COMMA NB>, concat_ref<AB CHDL_COMMA AA>,
                const const_slice_ref<TB CHDL_COMMA NB>&, const const_concat_ref<AB CHDL_COMMA AA>&,
                const slice_ref<TB CHDL_COMMA NB>&, const concat_ref<AB CHDL_COMMA AA>&)

CHDL_CONCAT_GEN(template <typename TB CHDL_COMMA unsigned NB CHDL_COMMA typename TA>, 
                const_slice_ref<TB CHDL_COMMA NB>, const_subscript_ref<TA>,
                slice_ref<TB CHDL_COMMA NB>, subscript_ref<TA>,
                const const_slice_ref<TB CHDL_COMMA NB>&, const const_subscript_ref<TA>&,
                const slice_ref<TB CHDL_COMMA NB>&, const subscript_ref<TA>&)

CHDL_CONCAT_GEN(template <typename TB CHDL_COMMA unsigned NB CHDL_COMMA unsigned NA>, 
                const_slice_ref<TB CHDL_COMMA NB>, ch_bitbase<NA>,
                slice_ref<TB CHDL_COMMA NB>, ch_bitbase<NA>,
                const const_slice_ref<TB CHDL_COMMA NB>&, const ch_bitbase<NA>&,
                const slice_ref<TB CHDL_COMMA NB>&, ch_bitbase<NA>&)

CHDL_CONCAT_GEN(template <typename TB CHDL_COMMA unsigned NB>, 
                const_slice_ref<TB CHDL_COMMA NB>, ch_logic,
                slice_ref<TB CHDL_COMMA NB>, ch_logic,
                const const_slice_ref<TB CHDL_COMMA NB>&, const ch_logic&,
                const slice_ref<TB CHDL_COMMA NB>&, ch_logic&)

//--

CHDL_CONCAT_GEN(template <typename TB CHDL_COMMA typename AB CHDL_COMMA typename AA>, 
                const_subscript_ref<TB>, const_concat_ref<AB CHDL_COMMA AA>,
                subscript_ref<TB>, concat_ref<AB CHDL_COMMA AA>,
                const const_subscript_ref<TB>&, const const_concat_ref<AB CHDL_COMMA AA>&,
                const subscript_ref<TB>&, const concat_ref<AB CHDL_COMMA AA>&)

CHDL_CONCAT_GEN(template <typename TB CHDL_COMMA typename TA CHDL_COMMA unsigned NA>, 
                const_subscript_ref<TB>, const_slice_ref<TA CHDL_COMMA NA>,
                subscript_ref<TB>, slice_ref<TA CHDL_COMMA NA>,
                const const_subscript_ref<TB>&, const const_slice_ref<TA CHDL_COMMA NA>&,
                const subscript_ref<TB>&, const slice_ref<TA CHDL_COMMA NA>&)

CHDL_CONCAT_GEN(template <typename TB CHDL_COMMA unsigned NA>, 
                const_subscript_ref<TB>, ch_bitbase<NA>,
                subscript_ref<TB>, ch_bitbase<NA>,
                const const_subscript_ref<TB>&, const ch_bitbase<NA>&,
                const subscript_ref<TB>&, ch_bitbase<NA>&)

CHDL_CONCAT_GEN(template <typename TB>, 
                const_subscript_ref<TB>, ch_logic,
                subscript_ref<TB>, ch_logic,
                const const_subscript_ref<TB>&, const ch_logic&,
                const subscript_ref<TB>&, ch_logic&)

//--

CHDL_CONCAT_GEN(template <unsigned NB CHDL_COMMA typename AB CHDL_COMMA typename AA>, 
                ch_bitbase<NB>, const_concat_ref<AB CHDL_COMMA AA>,
                ch_bitbase<NB>, concat_ref<AB CHDL_COMMA AA>,
                const ch_bitbase<NB>&, const const_concat_ref<AB CHDL_COMMA AA>&,
                ch_bitbase<NB>&, const concat_ref<AB CHDL_COMMA AA>&)

CHDL_CONCAT_GEN(template <unsigned NB CHDL_COMMA typename TA CHDL_COMMA unsigned NA>, 
                ch_bitbase<NB>, const_slice_ref<TA CHDL_COMMA NA>,
                ch_bitbase<NB>, slice_ref<TA CHDL_COMMA NA>,
                const ch_bitbase<NB>&, const const_slice_ref<TA CHDL_COMMA NA>&,
                ch_bitbase<NB>&, const slice_ref<TA CHDL_COMMA NA>&)

CHDL_CONCAT_GEN(template <unsigned NB CHDL_COMMA typename TA>, 
                ch_bitbase<NB>, const_subscript_ref<TA>,
                ch_bitbase<NB>, subscript_ref<TA>,
                const ch_bitbase<NB>&, const const_subscript_ref<TA>&,
                ch_bitbase<NB>&, const subscript_ref<TA>&)

CHDL_CONCAT_GEN(template <unsigned NB>, 
                ch_bitbase<NB>, ch_logic,
                ch_bitbase<NB>, ch_logic,
                const ch_bitbase<NB>&, const ch_logic&,
                ch_bitbase<NB>&, ch_logic&)

//--

CHDL_CONCAT_GEN(template <typename AB CHDL_COMMA typename AA>, 
                ch_logic, const_concat_ref<AB CHDL_COMMA AA>,
                ch_logic, concat_ref<AB CHDL_COMMA AA>,
                const ch_logic&, const const_concat_ref<AB CHDL_COMMA AA>&,
                ch_logic&, const concat_ref<AB CHDL_COMMA AA>&)

CHDL_CONCAT_GEN(template <typename TA CHDL_COMMA unsigned NA>, 
                ch_logic, const_slice_ref<TA CHDL_COMMA NA>,
                ch_logic, slice_ref<TA CHDL_COMMA NA>,
                const ch_logic&, const const_slice_ref<TA CHDL_COMMA NA>&,
                ch_logic&, const slice_ref<TA CHDL_COMMA NA>&)

CHDL_CONCAT_GEN(template <typename TA>, 
                ch_logic, const_subscript_ref<TA>,
                ch_logic, subscript_ref<TA>,
                const ch_logic&, const const_subscript_ref<TA>&,
                ch_logic&, const subscript_ref<TA>&)

CHDL_CONCAT_GEN(template <unsigned NA>, 
                ch_logic, ch_bitbase<NA>,
                ch_logic, ch_bitbase<NA>,
                const ch_logic&, const ch_bitbase<NA>&,
                ch_logic&, ch_bitbase<NA>&)

#undef CHDL_CONCAT_GEN

// null operators

template <unsigned N>
ch_bitv<N> ch_null() {
  return ch_bitv<N>(createNullNode(N));
}

// slice operators

template <unsigned N, unsigned M>
const_slice_ref<ch_bitbase<M>, N> ch_slice(const ch_bitbase<M>& in, size_t index = 0) {
  return in.template slice<N>(index);
}

template <unsigned N, unsigned M>
slice_ref<ch_bitbase<M>, N> ch_slice(ch_bitbase<M>& in, size_t index = 0) {
  return in.template slice<N>(index);
}

template <unsigned N, unsigned M>
const_slice_ref<ch_bitbase<M>, N> ch_aslice(const ch_bitbase<M>& in, size_t index = 0) {
  return in.template aslice<N>(index);
}

template <unsigned N, unsigned M>
slice_ref<ch_bitbase<M>, N> ch_aslice(ch_bitbase<M>& in, size_t index = 0) {
  return in.template aslice<N>(index);
}

// extend operators

template <unsigned D>
class zext_select {
public:
    template <unsigned M>
    ch_bitv<(M+D)> operator() (const ch_bitbase<M>& in) {
      return (ch_bitv<D>(0x0), in);
    }
};

template <>
class zext_select<0> {
public:
    template <unsigned M>
    ch_bitv<M> operator() (const ch_bitbase<M>& in) {
      return in;
    }
};

template <unsigned M, unsigned D>
class sext_pad {
public:
    ch_bitv<(M+D)> operator() (const ch_bitbase<M>& in) {
      return (in[M-1], ch_bitv<D>(0x0), ch_slice<M-1>(in, 1));
    }    
};

template <unsigned D>
class sext_pad<1, D> {
public:
    ch_bitv<(1+D)> operator() (const ch_logicbase& in) {
      return (in, ch_bitv<D>(0x0));
    }
};

template <unsigned D>
class sext_select {
public:
    template <unsigned M>
    ch_bitv<(M+D)> operator() (const ch_bitbase<M>& in) {
      return sext_pad<M, D>()(in);
    }
};

template <>
class sext_select<0> {
public:
    template <unsigned M>
    ch_bitv<M> operator() (const ch_bitbase<M>& in) {
      return in;
    }
};

template <unsigned N, unsigned M>
ch_bitv<N> ch_zext(const ch_bitbase<M>& in) {
  static_assert(N >= M, "invalid extend size");
  return zext_select<(N-M)>()(in);
}

template <unsigned N, unsigned M>
ch_bitv<N> ch_sext(const ch_bitbase<M>& in) {
  static_assert(N >= M, "invalid extend size");
  return sext_select<(N-M)>()(in);
}

// shuffle operators

template <unsigned N, unsigned I>
ch_bitv<N> ch_shuffle(const ch_bitbase<N>& in, const std::array<uint32_t, I>& indices) {
  static_assert((I % N) == 0, "invalid shuffle indices size");
  for (unsigned i = 0; i < I; ++i) {
    ch_aslice<(N / I)>(in) = indices[i];
  }
}

// literal operators

template <unsigned N>
ch_bitv<N> ch_lit(uint32_t value) {
  return ch_bitv<N>(value);
}

template <unsigned N>
ch_bitv<N> ch_lit(const std::initializer_list<uint32_t>& value) {
  return ch_bitv<N>(value);
}

}
