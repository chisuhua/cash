#pragma once

#include "logic.h"

namespace ch {
namespace internal {

lnodeimpl* createInputNode(const std::string& name,
                           uint32_t size,
                           const source_location& sloc);

lnodeimpl* createOutputNode(const std::string& name,
                            const lnode& src,
                            const source_location& sloc);

void bindInput(const lnode& src,
               const lnode& input,
               const source_location& sloc);

void bindOutput(const lnode& dst,
                const lnode& output,
                const source_location& sloc);

///////////////////////////////////////////////////////////////////////////////

template <typename T> class ch_logic_in;
template <typename T> class ch_logic_out;

template <typename T> class ch_scalar_in;
template <typename T> class ch_scalar_out;

template <typename T>
using ch_in = std::conditional_t<is_logic_only_v<T>,
                std::add_const_t<ch_logic_in<T>>, ch_scalar_in<T>>;

template <typename T>
using ch_out = std::conditional_t<is_logic_only_v<T>,
                  ch_logic_out<T>, std::add_const_t<ch_scalar_out<T>>>;

///////////////////////////////////////////////////////////////////////////////

template <typename T>
class ch_logic_in final : public T {
public:
  static_assert(is_logic_only_v<T>, "invalid type");
  using traits = mixed_logic_io_traits<ch_direction::in,
                                       ch_in<T>,
                                       ch_out<T>,
                                       ch_in<ch_scalar_t<T>>,
                                       T>;
  using base = T;

  explicit ch_logic_in(const std::string& name = "io", CH_SLOC)
     : base(logic_buffer(createInputNode(name, ch_width_v<T>, sloc))) {
    input_ = logic_accessor::data(*this);
  }

  template <typename U>
  explicit ch_logic_in(const ch_logic_out<U>& out, CH_SLOC)
    : base(logic_buffer(ch_width_v<T>, sloc)) {
    static_assert(is_logic_only_v<U>, "invalid type");
    static_assert(std::is_constructible_v<U, T>, "invalid type");
    bindOutput(logic_accessor::data(*this), out.output_, sloc);
  }

  explicit ch_logic_in(const ch_logic_in& other, CH_SLOC)
    : base(logic_accessor::buffer(other)) {
    CH_UNUSED(sloc);
  }

  ch_logic_in(const ch_logic_in&& other)
    : base(std::move(other))
    , input_(std::move(other.input_))
  {}

  template <typename U>
  void operator()(ch_logic_out<U>& out) const {
    static_assert(std::is_constructible_v<U, T>, "invalid type");
    out = *this;
  }

private:

  ch_logic_in& operator=(const ch_logic_in&) = delete;

  ch_logic_in& operator=(ch_logic_in&&) = delete;

  lnode input_;

  template <typename U> friend class ch_logic_out;
  template <typename U> friend class ch_scalar_in;
};

///////////////////////////////////////////////////////////////////////////////

template <typename T>
class ch_logic_out final : public T {
public:
  static_assert(is_logic_only_v<T>, "invalid type");
  using traits = mixed_logic_io_traits<ch_direction::out,
                                       ch_out<T>,
                                       ch_in<T>,
                                       ch_out<ch_scalar_t<T>>,
                                       T>;
  using base = T;
  using base::operator=;

  explicit ch_logic_out(const std::string& name = "io", CH_SLOC)
    : base(logic_buffer(ch_width_v<T>, sloc, name)) {
    output_ = createOutputNode(name, logic_accessor::data(*this), sloc);
  }

  template <typename U>
  explicit ch_logic_out(const ch_logic_in<U>& in, CH_SLOC)
    : base(logic_buffer(ch_width_v<T>, sloc)) {
    static_assert(is_logic_only_v<U>, "invalid type");
    static_assert(std::is_constructible_v<U, T>, "invalid type");
    bindInput(logic_accessor::data(*this), in.input_, sloc);
  }

  ch_logic_out(const ch_logic_out& other, CH_SLOC)
    : base(logic_accessor::buffer(other)) {
    CH_UNUSED(sloc);
  }

  ch_logic_out(ch_logic_out&& other)
    : base(std::move(other))
    , output_(std::move(other.output_))
  {}

  ch_logic_out& operator=(const ch_logic_out& other) {
    base::operator=(other);
    return *this;
  }

  ch_logic_out& operator=(ch_logic_out&& other) {
    base::operator=(std::move(other));
    output_ = std::move(other.output_);
    return *this;
  }

  template <typename U>
  void operator()(const ch_logic_in<U>& in) {
    static_assert(std::is_constructible_v<U, T>, "invalid type");
    *this = in;
  }

private:

  lnode output_;

  template <typename U> friend class ch_logic_in;
  template <typename U> friend class ch_scalar_out;
};

///////////////////////////////////////////////////////////////////////////////

class scalar_io_buffer : public scalar_buffer {
public:
  using base = scalar_buffer;

  explicit scalar_io_buffer(const lnode& io)
    : base(bitvector(), nullptr, 0, io.size()), io_(io)
  {}

  const bitvector& data() const override {
    return io_.data();
  }

  void read(uint32_t src_offset,
            void* out,
            uint32_t out_cbsize,
            uint32_t dst_offset,
            uint32_t length) const override {
    io_.data().read(src_offset, out, out_cbsize, dst_offset, length);
  }

  void write(uint32_t dst_offset,
             const void* in,
             uint32_t in_cbsize,
             uint32_t src_offset,
             uint32_t length) override {
    io_.data().write(dst_offset, in, in_cbsize, src_offset, length);
  }

  lnode io_;
};

///////////////////////////////////////////////////////////////////////////////

template <typename T>
class ch_scalar_in final : public T {
public:
  static_assert(is_scalar_only_v<T>, "invalid type");
  using traits = mixed_scalar_io_traits<ch_direction::in,
                                        ch_in<T>,
                                        ch_out<T>,
                                        ch_in<ch_logic_t<T>>,
                                        T>;
  using base = T;
  using base::operator=;

  template <typename U>
  explicit ch_scalar_in(const ch_logic_in<U>& in)
    : base(std::make_shared<scalar_io_buffer>(in.input_)) {
    static_assert(is_logic_only_v<U>, "invalid type");
    static_assert(ch_width_v<T> == ch_width_v<U>, "invalid size");
  }

  ch_scalar_in(const ch_scalar_in& other)
    : base(scalar_accessor::buffer(other))
  {}

  ch_scalar_in(ch_scalar_in&& other) : base(std::move(other)) {}

  ch_scalar_in& operator=(const ch_scalar_in& other) {
    base::operator=(other);
    return *this;
  }

  ch_scalar_in& operator=(ch_scalar_in&& other) {
    base::operator=(std::move(other));
    return *this;
  }

  template <typename U>
  void operator()(const ch_scalar_out<U>& out) {
    static_assert(std::is_constructible_v<U, T>, "invalid type");
    *this = out;
  }
};

///////////////////////////////////////////////////////////////////////////////

template <typename T>
class ch_scalar_out final : public T {
public:
  static_assert(is_scalar_only_v<T>, "invalid type");
  using traits = mixed_scalar_io_traits<ch_direction::out,
                                        ch_out<T>,
                                        ch_in<T>,
                                        ch_out<ch_logic_t<T>>,
                                        T>;
  using base = T;

  template <typename U>
  explicit ch_scalar_out(const ch_logic_out<U>& out)
    : base(std::make_shared<scalar_io_buffer>(out.output_)) {
    static_assert(is_logic_only_v<U>, "invalid type");
    static_assert(ch_width_v<T> == ch_width_v<U>, "invalid size");
  }

  explicit ch_scalar_out(const ch_scalar_out& other)
    : base(scalar_accessor::buffer(other))
  {}

  ch_scalar_out(const ch_scalar_out&& other) : base(std::move(other)) {}

  template <typename U>
  void operator()(ch_scalar_in<U>& out) const {
    static_assert(std::is_constructible_v<U, T>, "invalid type");
    out = *this;
  }

protected:

  ch_scalar_out& operator=(const ch_scalar_out&) = delete;

  ch_scalar_out& operator=(ch_scalar_out&&) = delete;
};

}
}
