#pragma once

#include "bit.h"

enum alu_flags {
  alu_unary   = 0 << 8,
  alu_binary  = 1 << 8,
  alu_tenary  = 2 << 8,
  alu_nary    = 3 << 8,

  alu_bitwise = 1 << 10,
  alu_compare = 2 << 10,
  alu_shift   = 3 << 10,
  alu_arithm  = 4 << 10,
  alu_reduce  = 5 << 10,
  alu_misc    = 6 << 10,

  alu_integer = 1 << 13,
  alu_fixed   = 2 << 13,
  alu_float   = 3 << 13,
  alu_double  = 4 << 13,
};

#define CH_ALUOP_TYPE(e, v) alu_op_##e = v,
#define CH_ALUOP_ENUM(m) \
  m(inv, 0 | alu_unary | alu_bitwise | alu_integer) \
  m(and, 1 | alu_binary | alu_bitwise | alu_integer) \
  m(or, 2 | alu_binary | alu_bitwise | alu_integer) \
  m(xor, 3 | alu_binary | alu_bitwise | alu_integer) \
  m(nand, 4 | alu_binary | alu_bitwise | alu_integer) \
  m(nor, 5 | alu_binary | alu_bitwise | alu_integer) \
  m(xnor, 6 | alu_binary | alu_bitwise | alu_integer) \
  m(andr, 7 | alu_unary | alu_reduce | alu_integer) \
  m(orr, 8 | alu_unary | alu_reduce | alu_integer) \
  m(xorr, 9 | alu_unary | alu_reduce | alu_integer) \
  m(nandr, 10 | alu_unary | alu_reduce | alu_integer) \
  m(norr, 11 | alu_unary | alu_reduce | alu_integer) \
  m(xnorr, 12 | alu_unary | alu_reduce | alu_integer) \
  m(shl, 13 | alu_binary | alu_shift | alu_integer) \
  m(shr, 14 | alu_binary | alu_shift | alu_integer) \
  m(add, 15 | alu_binary | alu_arithm | alu_integer) \
  m(sub, 16 | alu_binary | alu_arithm | alu_integer) \
  m(neg, 17 | alu_unary | alu_arithm | alu_integer) \
  m(mult, 18 | alu_binary | alu_arithm | alu_integer) \
  m(div, 19 | alu_binary | alu_arithm | alu_integer) \
  m(mod, 20 | alu_binary | alu_arithm | alu_integer) \
  m(eq, 21 | alu_binary | alu_compare | alu_integer) \
  m(ne, 22 | alu_binary | alu_compare | alu_integer) \
  m(lt, 23 | alu_binary | alu_compare | alu_integer) \
  m(gt, 24 | alu_binary | alu_compare | alu_integer) \
  m(le, 25 | alu_binary | alu_compare | alu_integer) \
  m(ge, 26 | alu_binary | alu_compare | alu_integer) \
  m(fadd, 27 | alu_binary | alu_arithm | alu_float) \
  m(fsub, 28 | alu_binary | alu_arithm | alu_float) \
  m(fmult, 29 | alu_binary | alu_arithm | alu_float) \
  m(fdiv, 30 | alu_binary | alu_arithm | alu_float)

#define CH_BINOP_GEN(func, op) \
  template <typename A, typename B, \
            CH_REQUIRES(deduce_type<A, B>::bitcount != 0), \
            CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value), \
            CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)> \
  const auto op(const A& a, const B& b) { \
    return func(a, b); \
  }

#define CH_UNARYOP_GEN(func, op) \
  template <unsigned N> \
  const auto op(const ch_bitbase<N>& in) { \
    return func(in); \
  }

#define CH_COMPAREOP_GEN(func, op) \
  template <typename A, typename B, \
            CH_REQUIRES(deduce_type<A, B>::bitcount != 0), \
            CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value), \
            CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)> \
  const auto op(const A& a, const B& b) { \
    return func(a, b); \
  }

#define CH_SHIFTOP_GEN(func, op) \
  template <typename A, typename B, \
            CH_REQUIRES(deduce_first_type<A, B>::bitcount != 0), \
            CH_REQUIRES(is_bit_convertible<A, deduce_first_type<A, B>::bitcount>::value), \
            CH_REQUIRES(is_bit_convertible<B, deduce_first_type<B, A>::bitcount>::value)> \
  const auto op(const A& a, const B& b) { \
    return func(a, b); \
  }

namespace cash {
namespace internal {

enum ch_alu_op {
  CH_ALUOP_ENUM(CH_ALUOP_TYPE)
};

lnodeimpl* createAluNode(ch_alu_op op, const lnode& lhs, const lnode& rhs);
lnodeimpl* createAluNode(ch_alu_op op, const lnode& in);

// compare operators

CH_COMPAREOP_GEN(ch_eq, operator==)
CH_COMPAREOP_GEN(ch_ne, operator!=)
CH_COMPAREOP_GEN(ch_lt, operator<)
CH_COMPAREOP_GEN(ch_gt, operator>)
CH_COMPAREOP_GEN(ch_le, operator<=)
CH_COMPAREOP_GEN(ch_ge, operator>=)

// logic operators

CH_BINOP_GEN(ch_and, operator&)
CH_BINOP_GEN(ch_or, operator|)
CH_BINOP_GEN(ch_xor, operator^)
CH_UNARYOP_GEN(ch_inv, operator~)

// arithmetic operators

CH_UNARYOP_GEN(ch_neg, operator-)
CH_BINOP_GEN(ch_add, operator+)
CH_BINOP_GEN(ch_sub, operator-)
CH_BINOP_GEN(ch_mult, operator*)
CH_BINOP_GEN(ch_div, operator/)
CH_BINOP_GEN(ch_mod, operator%)

// shift operators

CH_SHIFTOP_GEN(ch_sll, operator<<)
CH_SHIFTOP_GEN(ch_slr, operator>>)

///////////////////////////////////////////////////////////////////////////////

template <ch_alu_op op, unsigned N, typename A, typename B>
const auto OpBinary(const A& a, const B& b) {
  return make_bit<N>(createAluNode(op, get_lnode<A, N>(a), get_lnode<B, N>(b)));
}

template <ch_alu_op op, unsigned N, unsigned M, typename A, typename B>
const auto OpShift(const A& a, const B& b) {
  return make_bit<N>(createAluNode(op, get_lnode<A, N>(a), get_lnode<B, M>(b)));
}

template <ch_alu_op op, unsigned N, typename A, typename B>
const auto OpCompare(const A& a, const B& b) {
  return make_bit<1>(createAluNode(op, get_lnode<A, N>(a), get_lnode<B, N>(b)));
}

template <ch_alu_op op, unsigned N>
const auto OpUnary(const ch_bitbase<N>& a) {
  return make_bit<N>(createAluNode(op, get_lnode(a)));
}

template <ch_alu_op op, unsigned N>
const auto OpReduce(const ch_bitbase<N>& a) {
  return make_bit<1>(createAluNode(op, get_lnode(a)));
}

///////////////////////////////////////////////////////////////////////////////

template <unsigned N>
const auto ch_inv(const ch_bitbase<N>& a) {
  return OpUnary<alu_op_inv>(a);
}

template <typename A, typename B,
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0),
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value),
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)>
const auto ch_nand(const A& a, const B& b) {
  return OpBinary<alu_op_nand, deduce_type<A, B>::bitcount>(a, b);
}

template <typename A, typename B,
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0),
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value),
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)>
const auto ch_nor(const A& a, const B& b) {
  return OpBinary<alu_op_nor, deduce_type<A, B>::bitcount>(a, b);
}

template <typename A, typename B,
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0),
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value),
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)>
const auto ch_and(const A& a, const B& b) {
  return OpBinary<alu_op_and, deduce_type<A, B>::bitcount>(a, b);
}

template <typename A, typename B,
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0),
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value),
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)>
const auto ch_or(const A& a, const B& b) {
  return OpBinary<alu_op_or, deduce_type<A, B>::bitcount>(a, b);
}

template <typename A, typename B,
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0),
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value),
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)>
const auto ch_xor(const A& a, const B& b) {
  return OpBinary<alu_op_xor, deduce_type<A, B>::bitcount>(a, b);
}

template <typename A, typename B,
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0),
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value),
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)>
const auto ch_xnor(const A& a, const B& b) {
  return OpBinary<alu_op_xnor, deduce_type<A, B>::bitcount>(a, b);
}

///////////////////////////////////////////////////////////////////////////////

template <unsigned N>
const auto ch_andr(const ch_bitbase<N>& a) {
  return OpReduce<alu_op_andr>(a);
}

template <unsigned N>
const auto ch_orr(const ch_bitbase<N>& a) {
  return OpReduce<alu_op_orr>(a);
}

template <unsigned N>
const auto ch_xorr(const ch_bitbase<N>& a) {
  return OpReduce<alu_op_xorr>(a);
}

template <unsigned N>
const auto ch_nandr(const ch_bitbase<N>& a) {
  return OpReduce<alu_op_nandr>(a);
}

template <unsigned N>
const auto ch_norr(const ch_bitbase<N>& a) {
  return OpReduce<alu_op_norr>(a);
}

template <unsigned N>
const auto ch_xnorr(const ch_bitbase<N>& a) {
  return OpReduce<alu_op_xnorr>(a);
}

///////////////////////////////////////////////////////////////////////////////

inline const auto operator! (const ch_bitbase<1>& a) {
  return ch_inv(a);         
}

inline const auto operator&& (const ch_bitbase<1>& a, const ch_bitbase<1>& b) {
  return ch_and(a, b);      
}

inline const auto operator|| (const ch_bitbase<1>& a, const ch_bitbase<1>& b) {
  return ch_or(a, b);      
}

///////////////////////////////////////////////////////////////////////////////

template <typename A, typename B,
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0),
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value),
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)>
const auto ch_eq(const A& a, const B& b) {
  return OpCompare<alu_op_eq, deduce_type<A, B>::bitcount>(a, b);
}

template <typename A, typename B,
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0),
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value),
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)>
const auto ch_ne(const A& a, const B& b) {
  return OpCompare<alu_op_ne, deduce_type<A, B>::bitcount>(a, b);
}

template <typename A, typename B,
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0),
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value),
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)>
const auto ch_lt(const A& a, const B& b) {
  return OpCompare<alu_op_lt, deduce_type<A, B>::bitcount>(a, b);
}

template <typename A, typename B,
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0),
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value),
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)>
const auto ch_gt(const A& a, const B& b) {
  return OpCompare<alu_op_gt, deduce_type<A, B>::bitcount>(a, b);
}

template <typename A, typename B,
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0),
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value),
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)>
const auto ch_ge(const A& a, const B& b) {
  return OpCompare<alu_op_ge, deduce_type<A, B>::bitcount>(a, b);
}

template <typename A, typename B,
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0),
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value),
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)>
const auto ch_le(const A& a, const B& b) {
  return OpCompare<alu_op_le, deduce_type<A, B>::bitcount>(a, b);
}  

///////////////////////////////////////////////////////////////////////////////

template <typename A, typename B,
          CH_REQUIRES(deduce_first_type<A, B>::bitcount != 0),
          CH_REQUIRES(is_bit_convertible<A, deduce_first_type<A, B>::bitcount>::value),
          CH_REQUIRES(is_bit_convertible<B, deduce_first_type<B, A>::bitcount>::value)>
const auto ch_sll(const A& a, const B& b) {
  return OpShift<alu_op_shl, deduce_first_type<A, B>::bitcount, deduce_first_type<B, A>::bitcount>(a, b);
}

template <typename A, typename B,
          CH_REQUIRES(deduce_first_type<A, B>::bitcount != 0),
          CH_REQUIRES(is_bit_convertible<A, deduce_first_type<A, B>::bitcount>::value),
          CH_REQUIRES(is_bit_convertible<B, deduce_first_type<B, A>::bitcount>::value)>
const auto ch_slr(const A& a, const B& b) {
  return OpShift<alu_op_shr, deduce_first_type<A, B>::bitcount, deduce_first_type<B, A>::bitcount>(a, b);
}  

///////////////////////////////////////////////////////////////////////////////

template <unsigned N>
const auto ch_neg(const ch_bitbase<N>& a) {
  return OpUnary<alu_op_neg>(a);
}

template <typename A, typename B, \
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0), \
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value), \
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)> \
const auto ch_add(const A& a, const B& b) {
  return OpBinary<alu_op_add, deduce_type<A, B>::bitcount>(a, b);
}

template <typename A, typename B, \
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0), \
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value), \
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)> \
const auto ch_sub(const A& a, const B& b) {
  return OpBinary<alu_op_sub, deduce_type<A, B>::bitcount>(a, b);
}

template <typename A, typename B, \
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0), \
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value), \
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)> \
const auto ch_mult(const A& a, const B& b) {
  return OpBinary<alu_op_mult, deduce_type<A, B>::bitcount>(a, b);
}

template <typename A, typename B, \
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0), \
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value), \
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)> \
const auto ch_div(const A& a, const B& b) {
  return OpBinary<alu_op_div, deduce_type<A, B>::bitcount>(a, b);
}

template <typename A, typename B, \
          CH_REQUIRES(deduce_type<A, B>::bitcount != 0), \
          CH_REQUIRES(is_bit_convertible<A, deduce_type<A, B>::bitcount>::value), \
          CH_REQUIRES(is_bit_convertible<B, deduce_type<A, B>::bitcount>::value)> \
const auto ch_mod(const A& a, const B& b) {
  return OpBinary<alu_op_mod, deduce_type<A, B>::bitcount>(a, b);
}

///////////////////////////////////////////////////////////////////////////////

template <typename I, typename S,
          CH_REQUIRES(is_bit_convertible<I>::value),
          CH_REQUIRES(is_bit_convertible<S>::value)>
const auto ch_mux(const I& in, const S& sel) {
  //return make_bit<(I::bitcount >> S::bitcount)>(
  //      createAluNode(alu_op_mux, get_lnode(in), get_lnode(sel)));
  CH_TODO();
}

}
}

#undef CH_BINOP_GEN0
#undef CH_BINOP_GEN1
#undef CH_BINOP_GEN2

#undef CH_UNARYOP_GEN

#undef CH_COMPAREOP_GEN0
#undef CH_COMPAREOP_GEN1

#undef CH_SHIFTOP_GEN0
#undef CH_SHIFTOP_GEN1
#undef CH_SHIFTOP_GEN2
