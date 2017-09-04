#pragma once

#include "bit.h"
#include "select.h"

namespace cash {
namespace internal {

class regimpl;
class latchimpl;

lnodeimpl* createRegNode(const lnode& next, const lnode& init);
lnodeimpl* createLatchNode(const lnode& next,
                           const lnode& init,
                           const lnode& enable,
                           const lnode& reset);

lnodeimpl* createReadyNode(const lnode& node);
lnodeimpl* createValidNode(const lnode& node);

const ch_bit<1> ch_getClock();
void ch_pushClock(const ch_bitbase<1>& clk);
void ch_popClock();

const ch_bit<1> ch_getReset();
void ch_pushReset(const ch_bitbase<1>& reset);
void ch_popReset();

template <unsigned N>
const auto ch_ready(const ch_bitbase<N>& x) {
  return make_bit<1>(createReadyNode(get_node(x)));
}

template <unsigned N>
const auto ch_valid(const ch_bitbase<N>& x) {
  return make_bit<1>(createValidNode(get_node(x)));
}

template <unsigned N>
const auto ch_reg(const ch_bitbase<N>& next, const ch_bitbase<N>& init) {
  return make_bit<N>(createRegNode(get_node(next), get_node(init)));
}

template <unsigned N>
const auto ch_reg(const ch_bitbase<N>& next, const ch_bit<N>& init = false) {
  return ch_reg<N>(next, reinterpret_cast<const ch_bitbase<N>&>(init));
}

template <unsigned N>
const auto ch_reg(const ch_bit<N>& next, const ch_bitbase<N>& init) {
  return ch_reg<N>(reinterpret_cast<const ch_bitbase<N>&>(next), init);
}

template <unsigned N>
const auto ch_reg(const ch_bit<N>& next, const ch_bit<N>& init) {
  return ch_reg<N>(reinterpret_cast<const ch_bitbase<N>&>(next), 
                   reinterpret_cast<const ch_bitbase<N>&>(init));
}

template <unsigned N>
const auto ch_latch(const ch_bitbase<N>& next,
                    const ch_bitbase<1>& enable,
                    const ch_bitbase<N>& init) {
  return make_bit<N>(createLatchNode(get_node(next),
                                     get_node(init),
                                     get_node(enable),
                                     get_node(ch_getReset())));
}

template <unsigned N>
const auto ch_latch(const ch_bitbase<N>& next,
                    const ch_bitbase<1>& enable,
                    const ch_bit<N>& init = false) {
  return ch_latch<N>(next,
                     enable,
                     reinterpret_cast<const ch_bitbase<N>&>(init));
}

template <unsigned N>
const auto ch_latch(const ch_bitbase<N>& next,
                    const ch_bit<1>& enable,
                    const ch_bitbase<N>& init) {
  return ch_latch<N>(next,
                     reinterpret_cast<const ch_bitbase<1>&>(enable),
                     init);
}

template <unsigned N>
const auto ch_latch(const ch_bitbase<N>& next,
                    const ch_bit<1>& enable = true,
                    const ch_bit<N>& init = false) {
  return ch_latch<N>(next,
                     reinterpret_cast<const ch_bitbase<1>&>(enable),
                     reinterpret_cast<const ch_bitbase<N>&>(init));
}

template <unsigned N>
const auto ch_latch(const ch_bit<N>& next,
                    const ch_bitbase<1>& enable,
                    const ch_bitbase<N>& init) {
  return ch_latch<N>(reinterpret_cast<const ch_bitbase<N>&>(next),
                     enable,
                     init);
}

template <unsigned N>
const auto ch_latch(const ch_bit<N>& next,
                    const ch_bitbase<1>& enable,
                    const ch_bit<N>& init = false) {
  return ch_latch<N>(reinterpret_cast<const ch_bitbase<N>&>(next), 
                     enable, 
                     reinterpret_cast<const ch_bitbase<N>&>(init));
}

template <unsigned N>
const auto ch_latch(const ch_bit<N>& next,
                    const ch_bit<1>& enable,
                    const ch_bitbase<N>& init) {
  return ch_latch<N>(reinterpret_cast<const ch_bitbase<N>&>(next), 
                     reinterpret_cast<const ch_bitbase<1>&>(enable),
                     init);
}

template <unsigned N>
const auto ch_latch(const ch_bit<N>& next,
                    const ch_bit<1>& enable = true,
                    const ch_bit<N>& init = false) {
  return ch_latch<N>(reinterpret_cast<const ch_bitbase<N>&>(next), 
                     reinterpret_cast<const ch_bitbase<1>&>(enable),
                     reinterpret_cast<const ch_bitbase<N>&>(init));
}

}
}
