#pragma once

#include <cash.h>
#include <htl/counter.h>
#include <htl/decoupled.h>

namespace ch {
namespace htl {

using namespace ch::logic;

template <typename T, unsigned N>
struct ch_queue {
  using value_type = T;
  static constexpr uint32_t max_size   = N;
  static constexpr uint32_t size_width = log2ceil(N+1);
  static_assert(N > 1, "invalid size");

  __io (
    (ch_enq_io<T>) enq,
    (ch_deq_io<T>) deq,
    __out(ch_uint<size_width>) size
  );

  void describe() {    
    auto reading = io.deq.ready && io.deq.valid;
    auto writing = io.enq.valid && io.enq.ready;

    ch_reg<ch_uint<size_width>> counter(0);
    __if (writing && !reading) {
      counter->next = counter + 1;
    }__elif (reading && !writing) {
      counter->next = counter - 1;
    };

    ch_reg<ch_bool> empty(true);
    __if (counter == 1 && reading && !writing) {
      empty->next = true;
    }__elif (writing && !reading) {
      empty->next = false;
    };

    ch_reg<ch_bool> full(false);
    __if (counter == (N-1) && writing && !reading) {
      full->next = true;
    }__elif (reading && !writing) {
      full->next = false;
    };

    ch_counter<N> rd_ptr(reading);
    ch_counter<N> wr_ptr(writing);

    ch_mem<T, N> mem;
    mem.write(wr_ptr.value, io.enq.data, writing);

    ch_reg<T> data_out;
    __if (writing && (empty || (1 == counter && reading))) {
      data_out->next = io.enq.data;
    }__elif (reading) {
      data_out->next = mem.read(rd_ptr.value + 1);
    }__else {
      data_out->next = mem.read(rd_ptr.value);
    };

    io.deq.data  = data_out;
    io.deq.valid = !empty;
    io.enq.ready = !full;
    io.size      = counter;
  }
};

template <typename T, unsigned N>
struct ch_llqueue {
  using value_type = T;
  static constexpr uint32_t max_size   = N;
  static constexpr uint32_t size_width = log2ceil(N+1);

  __io (
    (ch_enq_io<T>) enq,
    (ch_deq_io<T>) deq,
    __out(ch_uint<size_width>) size
  );

  void describe() {
    auto reading = io.deq.ready && io.deq.valid;
    auto writing = io.enq.valid && io.enq.ready;

    if constexpr (N == 1) {
      ch_reg<ch_uint1> size(0);

      __if (writing && !reading) {
        size->next = 1;
      }__elif (reading && !writing) {
        size->next = 0;
      };

      ch_reg<T> data;
      __if (writing) {
        data->next = io.enq.data;
      };

      io.deq.data  = data;
      io.deq.valid = size;
      io.enq.ready = !size;
      io.size      = size;
    } else {
      ch_reg<ch_uint<size_width>> size(0);
      ch_reg<ch_bit<N+1>> ptr(1);

      __if (writing && !reading) {
        size->next = size + 1;
        ptr->next = ch_cat(ch_slice<N>(ptr, 0), 0_b);
      }__elif (reading && !writing) {
        size->next = size - 1;
        ptr->next = ch_cat(0_b, ch_slice<N>(ptr, 1));
      };

      ch_bit<N> data_upd, data_new;
      for (unsigned i = 0; i < N; ++i) {
        data_upd[i] = reading | (writing & ptr[i]);
        data_new[i] = (!reading) | ptr[i+1];
      }

      std::array<ch_reg<T>, N> data;
      for (unsigned i = 0; i < N-1; ++i) {
        data[i]->next = ch_sel(data_upd[i], ch_sel(data_new[i], io.enq.data, data[i+1]), data[i]);
      }
      data[N-1]->next = ch_sel(data_upd[N-1], io.enq.data, data[N-1]);

      io.deq.data  = data[0];
      io.deq.valid = !ptr[0];
      io.enq.ready = !ptr[N];
      io.size      = size;
    }
  }
};

}
}
