#include <htl/float.h>
#include "common.h"

using namespace ch::htl;

namespace {

  template <unsigned M, unsigned N, unsigned TF, unsigned Delay>
  struct FMultTest {
    __io (
      __in (ch_float<M, N, TF>) lhs,
      __in (ch_float<M, N, TF>) rhs,
      __out (ch_float<M, N, TF>) out
    );

    void describe() {
      auto a = ufadd<M, N, TF, Delay>(io.lhs, io.rhs);
      auto b = ufsub<M, N, TF, Delay>(a, io.rhs);
      auto c = ufmul<M, N, TF, Delay>(b, io.lhs);
      auto d = ufmul<M, N, TF, Delay>(c, a);
      io.out = d;
      //ch_println("{0}: clk={1}, rst={2}, a={3}, b={4}, c={5}, d={6}", ch_now(), ch_clock(), ch_reset(), a, b, c, d);
    }
  };
}

#define M 23
#define E 8
#define TF 0
#define SFLOAT ch_sfloat<M, E, TF>
#define FLOAT ch_float<M, E, TF>

TEST_CASE("ufloat", "[ufloat]") {
  SECTION("sbasic", "[sbasic]") {
    TEST([]()->ch_bool {
      SFLOAT x(1.0f), y(1.0f);
      //int x = 0;
      //int y = 0;
      ch_println("{0}: x={1}, y={2}\n", ch_now(), static_cast<ch_bit32>(x), static_cast<ch_bit32>(y));
      return (x == y);
    });

    TEST([]()->ch_bool {
      SFLOAT x(0.0f), y(1.0f);
      ch_println("{0}: x={1}, y={2}\n", ch_now(), static_cast<ch_bit32>(x), static_cast<ch_bit32>(y));
      return (x != y);
    });

    TEST([]()->ch_bool {
      SFLOAT x(0.0f);
      ch_bit32 y(0);
      ch_println("{0}: x={1}, y={2}\n", ch_now(), static_cast<ch_bit32>(x), y);
      return (x.as_bit() == y);
    });

    TEST([]()->ch_bool {
      ch_sbit32 x(0);
      SFLOAT y(x);
      return (static_cast<float>(y) == 0.0f);
    });

    TEST([]()->ch_bool {
      ch_sbit32 x(0x3f800000_h);
      SFLOAT y(x);
      return (static_cast<float>(y) == 1.0f);
    });

    TEST([]()->ch_bool {
      SFLOAT x(0.5f);
      return (x.as_bit() == 0x3f000000_h);
    });

    TEST([]()->ch_bool {
      SFLOAT x(1.0f);
      return (x.as_bit() == 0x3f800000_h);
    });

    TEST([]()->ch_bool {
      SFLOAT x(2.0f);
      return (x.as_bit() == 0x40000000_h);
    });
  }

  SECTION("sarithmetic", "[sarithmetic]") {
    TEST([]()->ch_bool {
      SFLOAT x(0.5f), y(0.0f), z;
      z = x * y;
      std::cout << z << "=" << x  << "*" << y << "\n";
      std::cout << z.as_bit() << "=" << x.as_bit()  << "*" << y.as_bit() << "\n";
      /*
      ch_println("{0}: x={1}, y={2}, z={3}", ch_now(), static_cast<float>(x),
                                                        static_cast<float>(x),
                                                        static_cast<float>(x));
                                                        */
      return (static_cast<float>(z) == 0.0f);
    }, 1);

    TEST([]()->ch_bool {
      SFLOAT x(0.5f), y(0.5f), z;
      z = x * y;
      std::cout << z << "=" << x  << "*" << y << "\n";
      std::cout << z.as_bit() << "=" << x.as_bit()  << "*" << y.as_bit() << "\n";
      //ch_println("{0}: clk={1}, rst={2}, z={3}", ch_now(), ch_clock(), ch_reset(), z);
      return (z.as_bit() == 0x3e800000_h);
    }, 1);

    TEST([]()->ch_bool {
      SFLOAT x(0.5f), y(1.0f), z;
      z = x * y;
      std::cout << z << "=" << x  << "*" << y << "\n";
      return (z.as_bit() == 0x3f000000_h);
    }, 1);

    TEST([]()->ch_bool {
      SFLOAT x(0.5f), y(2.0f), z;
      z = x * y;
      std::cout << z << "=" << x  << "*" << y << "\n";
      return (z.as_bit() == 0x3f800000_h);
    }, 1);

    TEST([]()->ch_bool {
      SFLOAT x(0.5f), y(1.5f), z;
      SFLOAT zz(2.0f);
      z = x + y;
      std::cout << z << ":" << z.as_bit() << "=" << x << ":" << x.as_bit() << "+" << y << ":" << y.as_bit() << "\n";
      std::cout << " zz:" << zz.as_bit() << "\n";
      return (z.as_bit() == 0x40000000_h);
    }, 1);

    TEST([]()->ch_bool {
      SFLOAT x(2.5f), y(0.5f), z;
      z = x - y;
      std::cout << z << "=" << x  << "-" << y << "\n";
      return (z.as_bit() == 0x40000000_h);
    }, 1);

    TEST([]()->ch_bool {
      SFLOAT x(2.5f), y(0.5f), z;
      z = x / y;
      SFLOAT zz(5.0f);
      std::cout << z << "=" << x  << "/" << y << "\n";
      std::cout << " zz:" << zz.as_bit() << "\n";
      return (z.as_bit() == zz.as_bit());
    }, 1);

    TEST([]()->ch_bool {
      SFLOAT a(0.1f);
      //std::cout << a;
      //ch_println("a={0:f}", static_cast<float>(a));
      ch_println("a={}", static_cast<ch_bit32>(a));
      return ch_true;
    });
  }
  // logic
  SECTION("basic", "[basic]") {
    TEST([]()->ch_bool {
      FLOAT x(1.0f), y(1.0f);
      //int x = 0;
      //int y = 0;
      //ch_println("{0}: x={1}, y={2}\n", ch_now(), static_cast<ch_bit32>(x), static_cast<ch_bit32>(y));
      ch_println("{}: x={}, y={}", ch_now(), x.as_bit(), y.as_bit());
      return (x == y);
    });

    TEST([]()->ch_bool {
      FLOAT x(0.0f), y(1.0f);
      ch_println("{}: x={}, y={}", ch_now(), static_cast<ch_bit32>(x), static_cast<ch_bit32>(y));
      return (x != y);
    });

    TEST([]()->ch_bool {
      FLOAT x(0.0f);
      ch_bit32 y(0);
      ch_println("{}: x={}, y={}", ch_now(), static_cast<ch_bit32>(x), y);
      return (x.as_bit() == y);
    });

    TEST([]()->ch_bool {
      ch_bit32 x(0);
      FLOAT y(x);
      FLOAT yy(0.0f);
      ch_println("{}: x={}, y={}", ch_now(), x, static_cast<ch_bit32>(y));
      return (y == yy);
    });

    TEST([]()->ch_bool {
      ch_bit32 x(0x3f800000_h);
      FLOAT y(x);
      ch_println("{}: x={}, y={}", ch_now(), x, y);
      return (y == FLOAT(1.0f));
    });

    TEST([]()->ch_bool {
      FLOAT x(0.5f);
      return (x.as_bit() == 0x3f000000_h);
    });

    TEST([]()->ch_bool {
      FLOAT x(1.0f);
      return (x.as_bit() == 0x3f800000_h);
    });

    TEST([]()->ch_bool {
      FLOAT x(2.0f);
      return (x.as_bit() == 0x40000000_h);
    });
  }


  SECTION("arithmetic", "[arithmetic]") {
    TEST([]()->ch_bool {
      FLOAT x(0.5f), y(0.0f), z;
      z = ufmul<M, E, TF, 1>(x, y);
      //ch_println("{0}: clk={1}, rst={2}, z={3}", ch_now(), ch_clock(), ch_reset(), z);
      return (z == FLOAT(0.0f));
    }, 1);

    TEST([]()->ch_bool {
      FLOAT x(0.5f), y(0.5f), z;
      z = ufmul<M, E, TF, 1>(x, y);
      //ch_println("{0}: clk={1}, rst={2}, z={3}", ch_now(), ch_clock(), ch_reset(), z);
      return (z.as_bit() == 0x3e800000_h);
    }, 1);

    TEST([]()->ch_bool {
      FLOAT x(0.5f), y(1.0f), z;
      z = ufmul<M, E, TF, 1>(x, y);
      return (z.as_bit() == 0x3f000000_h);
    }, 1);

    TEST([]()->ch_bool {
      FLOAT x(0.5f), y(2.0f), z;
      z = ufmul<M, E, TF, 1>(x, y);
      return (z.as_bit() == 0x3f800000_h);
    }, 1);

    TEST([]()->ch_bool {
      FLOAT x(0.5f), y(1.5f), z;
      z = ufadd<M, E, TF, 1>(x, y);
      return (z.as_bit() == 0x40000000_h);
    }, 1);

    TEST([]()->ch_bool {
      FLOAT x(2.5f), y(0.5f), z;
      z = ufsub<M, E, TF, 1>(x, y);
      return (z.as_bit() == 0x40000000_h);
    }, 1);

    TEST([]()->ch_bool {
      FLOAT x(0.5f), y(0.5f), z, e;
      z = ufmul<M, E, TF, 5>(x, y);
      // FLOAT zz(0x3e800000_h);
      // e = 0x3e800000_h;
      e = ch_case<FLOAT>(ch_now(), (2+5*2), 0x3e800000_h)(z);
      // e = ch_case<FLOAT>(ch_now(), (2+5*2), zz)(z);
      ch_println("{}: clk={}, rst={}, z={}, e={}", ch_now(), ch_clock(), ch_reset(), z, e);
      return (z == e);
    }, 5);
#if 0
    TEST([]()->ch_bool {
      FLOAT x(0.5f), y(0.5f), z, e;
      z = ufadd<M, E, 7>(x, y);
      e = ch_case<FLOAT>(ch_now(), (2+7*2), 0x3e800000_h)(z);
      //ch_println("{0}: clk={1}, rst={2}, z={3}, e={4}", ch_now(), ch_clock(), ch_reset(), z, e);
      return (z == e);
    }, 7);
#endif
    TEST([]()->ch_bool {
      FLOAT a(0.1f);
      //ch_println("a={0:f}", a);
      return ch_true;
    });
  }

/*
  SECTION("verilog", "[verilog]") {
    TESTX([]()->bool {
      ch_device<FMultTest<M, E, 2>> device;
      device.io.lhs = 0.5f;
      device.io.rhs = 0.5f;
      ch_simulator sim(device);
      sim.run(2*(2+2*2*4));
      ch_toVerilog("fmultest.v", device);
      float ret(device.io.out);
      //std::cout << "ret=" << ret << std::endl;
      return (0.25f == ret);
    });
  }
*/
}
