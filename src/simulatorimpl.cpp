#include "simulatorimpl.h"
#include "simulator.h"
#include "deviceimpl.h"
#include "litimpl.h"
#include "ioimpl.h"

using namespace cash::internal;

void clock_driver::add_node(lnodeimpl* node) {
  node->set_bool(0, value_);
  nodes_.push_back(node);
}

void clock_driver::flip() {
  value_ = !value_;
  for (auto node : nodes_) {
    node->set_bool(0, value_);
  }
}

///////////////////////////////////////////////////////////////////////////////

simulatorimpl::simulatorimpl(const std::initializer_list<const ch_device*>& devices)
  : initialized_(false)
  , clk_(true)
  , reset_(false) {
  for (auto device : devices) {
    context* ctx = get_ctx(*device);
    auto ret = contexts_.emplace(ctx);
    if (ret.second)
      ctx->acquire();
  }
}

simulatorimpl::~simulatorimpl() {
  for (auto ctx : contexts_) {
    ctx->release();
  }
}

void simulatorimpl::add_device(const ch_device& device) {
  context* ctx = get_ctx(device);
  auto ret = contexts_.emplace(ctx);
  if (ret.second) {
    ctx->acquire();
  }
}

void simulatorimpl::ensureInitialize() {
  // bind clocks
  for (auto ctx : contexts_) {
    auto clk = ctx->get_default_clk();
    if (clk) {
      clk_.add_node(clk);
    }
    auto reset = ctx->get_default_reset();
    if (reset) {
      reset_.add_node(reset);
    }
  }
}

void simulatorimpl::tick(ch_tick t) {
  // ensure initialized
  if (!initialized_) {
    this->ensureInitialize();
    initialized_ = true;
  }

  // evaluate all contexts
  for (auto ctx : contexts_) {
    ctx->tick(t);
  }
  for (auto ctx : contexts_) {
    ctx->tick_next(t);
  }
  for (auto ctx : contexts_) {
    ctx->eval(t);

  #ifndef NDEBUG
    int dump_ast_level = platform::self().get_dump_ast();
    if (2 == dump_ast_level) {
      std::cerr << "tick " << t << ":" << std::endl;
      ctx->dump_ast(std::cerr, 2);
    }
  #endif
  }
}

void simulatorimpl::run(const std::function<bool(ch_tick t)>& callback) {
  ch_tick start = this->reset(0);
  for (ch_tick t = start; callback(t - start);) {
    t = this->step(t);
  }
}

void simulatorimpl::run(ch_tick ticks) {
  ch_tick t = this->reset(0);
  for (; t < ticks;) {
    t = this->step(t);
  }
}

ch_tick simulatorimpl::reset(ch_tick t) {
  // ensure initialized
  if (!initialized_) {
    this->ensureInitialize();
    initialized_ = true;
  }

  if (!reset_.is_empty()) {
    reset_.flip();
    t = this->step(t);
    reset_.flip();
  }

  return t;
}

ch_tick simulatorimpl::step(ch_tick t) {
  if (!clk_.is_empty()) {
    for (int i = 0; i < 2; ++i) {
      clk_.flip();
      this->tick(t++);
    }
  } else {
    this->tick(t++);
  }
  return t;
}

///////////////////////////////////////////////////////////////////////////////

ch_simulator::ch_simulator(const std::initializer_list<const ch_device*>& devices) {
  impl_ = new simulatorimpl(devices);
  impl_->acquire();
}

ch_simulator::ch_simulator(simulatorimpl* impl) : impl_(impl) {
  if (impl)
    impl->acquire();
}

ch_simulator::ch_simulator(const ch_simulator& simulator) : impl_(simulator.impl_) {
  if (impl_)
    impl_->acquire();
}

ch_simulator::~ch_simulator() {
  if (impl_)
    impl_->release();
}

ch_simulator& ch_simulator::operator=(const ch_simulator& simulator) {
  if (simulator.impl_)
    simulator.impl_->acquire();
  if (impl_)
    impl_->release();
  impl_ = simulator.impl_;
  return *this;
}

void ch_simulator::add_device(const ch_device& device) {
  impl_->add_device(device);
}

void ch_simulator::tick(ch_tick t) { 
  impl_->tick(t);
}

void ch_simulator::run(const std::function<bool(ch_tick t)>& callback) {
  impl_->run(callback);
}

void ch_simulator::run(ch_tick ticks) {
  impl_->run(ticks);
}

ch_tick ch_simulator::reset(ch_tick t) {
  return impl_->reset(t);
}

ch_tick ch_simulator::step(ch_tick t) {
  return impl_->step(t);
}
