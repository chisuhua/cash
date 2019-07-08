#pragma once

#include <cash.h>

namespace ch {
namespace htl {

using namespace ch::logic;

template <typename T>
__inout (ch_valid_in, (
  __in (T) data,
  __in (ch_bool) valid
));

template <typename T> using ch_valid_out = ch_flip_io<ch_valid_in<T>>;

template <typename T> using ch_valid_t = ch_logic_t<ch_valid_in<T>>;

template <typename T = void>
__inout (ch_enq_io, ch_valid_in<T>, (
  __out (ch_bool) ready
));

namespace detail {
  __inout (enq_io_void, (
    __in (ch_bool) valid,
    __out (ch_bool) ready
  ));
}

template <>
struct ch_enq_io<void> : public detail::enq_io_void {
  using detail::enq_io_void::enq_io_void;
  using detail::enq_io_void::operator=;
};

template <typename T = void> using ch_deq_io = ch_flip_io<ch_enq_io<T>>;

}
}
