#include "common.h"
#include <limits>

static void begin_test() {
  static int test_number = 0;
  std::cout << "running test #" << test_number++ << " ..." << std::endl;
}

static void end_test() {
  //--
}

struct TestRunner {
  __io (
    __out (ch_bool) out
  );

  TestRunner(const std::function<ch_bool ()>& test)
    : test_(test)
  {}

  void describe() {
    io.out = test_();
  }

  const std::function<ch_bool()> test_;
};

struct TestRunner1 {
  __io (
    __in (ch_int8) in1,
    __out (ch_bool) out
  );

  TestRunner1(const std::function<ch_bool (const ch_int8&)>& test)
    : test_(test)
  {}

  void describe() {
    io.out = test_(io.in1);
  }

  const std::function<ch_bool(const ch_int8&)> test_;
};

struct TestRunner2 {
  __io (
    __in (ch_int8) in1,
    __in (ch_int8) in2,
    __out (ch_bool) out
  );

  TestRunner2(const std::function<ch_bool (const ch_int8&, const ch_int8&)>& test)
    : test_(test)
  {}

  void describe() {
    io.out = test_(io.in1, io.in2);
  }

  const std::function<ch_bool(const ch_int8&, const ch_int8&)> test_;
};

RetCheck& RetCheck::operator&=(bool value) {
  assert(value);
  count_ = value ? (count_ + 1) : std::numeric_limits<int>::min();
  return *this;
}

bool TEST(const std::function<ch_bool ()>& test, ch_tick cycles) {
  ch_device<TestRunner> device(test);
  ch_simulator sim(device);

  begin_test();

  auto ticks = (0 == cycles) ? 1 : (cycles * 2);

  sim.run([&](ch_tick t)->bool {
    std::cout << "t" << t << ": ret=" << device.io.out << std::endl;
    if (t > 0 && !(bool)device.io.out)
      return false;
    return (t < ticks);
  });

  bool bRet = (bool)device.io.out;
  assert(bRet);
  end_test();
  return bRet;
}

bool TEST1(const std::function<ch_bool (const ch_int8&)>& test, ch_tick cycles) {
  ch_device<TestRunner1> device(test);
  ch_simulator sim(device);

  begin_test();

  auto ticks = (0 == cycles) ? 1 : (cycles * 2);

  device.io.in1 = 1;

  sim.run([&](ch_tick t)->bool {
    std::cout << "t" << t << ": ret=" << device.io.out << std::endl;
    if (t > 0 && !(bool)device.io.out)
      return false;
    return (t < ticks);
  });

  bool bRet = (bool)device.io.out;
  assert(bRet);
  end_test();
  return bRet;
}

bool TEST2(const std::function<ch_bool (const ch_int8&, const ch_int8&)>& test, ch_tick cycles) {
  ch_device<TestRunner2> device(test);
  ch_simulator sim(device);

  begin_test();

  auto ticks = (0 == cycles) ? 1 : (cycles * 2);

  device.io.in1 = 1;
  device.io.in2 = 2;

  sim.run([&](ch_tick t)->bool {
    std::cout << "t" << t << ": ret=" << device.io.out << std::endl;
    if (t > 0 && !(bool)device.io.out)
      return false;
    return (t < ticks);
  });

  bool bRet = (bool)device.io.out;
  assert(bRet);
  end_test();
  return bRet;
}

bool TESTX(const std::function<bool()>& test) {
  begin_test();
  bool bRet = test();
  assert(bRet);
  end_test();
  return bRet;
}

bool checkVerilog(const std::string& file) {
  int ret = system(stringf("iverilog %s -o %s.iv", file.c_str(), file.c_str()).c_str())
          | system(stringf("! vvp %s.iv | grep 'ERROR' || false", file.c_str()).c_str());
  assert(0 == ret );
  return (0 == ret);
}
