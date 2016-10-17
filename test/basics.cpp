#include "common.h"

TEST_CASE("basic tests", "[basic]") {   
  
  SECTION("test literals", "[literal]") {        
    TEST([]()->ch_logic {
      ch_bitv<4> a(0x5);
      return (a == 0101_b);
    });
    TEST([]()->ch_logic {
      ch_bitv<3> a(0x5);
      return (a == 5_o);
    });
    TEST([]()->ch_logic {
      ch_bitv<4> a(0x5);
      return (a == 5_d);
    });
    TEST([]()->ch_logic {
      ch_bitv<4> a(0x5);
      return (a == 5_h);
    });
    TEST([]()->ch_logic {
      ch_bitv<4> a(0x5);
      return (a == 5e4_h);
    });
    TEST([]()->ch_logic {
      ch_bitv<64> a(0x5);
      return (a == 5e64_h);
    });
  }
  
  SECTION("test subscript", "[subscript]") {
    TEST([]()->ch_logic {
      ch_bit4 a(1010_b);
      return (a[0] == a[2] && a[1] == a[3]);
    });
    TEST([]()->ch_logic {
       ch_bit4 a(0110_b);
       return (a[0] == a[3] && a[1] == a[2]);
    });    
    TEST([]()->ch_logic {
      ch_bit4 a;
      ch_logic x = a[3];
      ch_logic y = a[2];
      x = 1_b;
      y = 0_b;
      return (a[2] == 0_b && a[3] == 1_b);
    });
  }
  
  SECTION("test slice", "[slice]") {
    TEST([]()->ch_logic {
      ch_bit4 a(1100_b);
      ch_bit2 c = a.slice<2>(1) ^ 01_b;
      return (c == 11_b);
    });
    TEST([]()->ch_logic {
      ch_bit4 a(1100_b);
      ch_bit2 c = a.aslice<2>(1) ^ 01_b;
      return (c == 10_b);
    });
    TEST([]()->ch_logic {
      ch_bit4 a;
      a.slice<3>(0) = '0';
      a[3] = '1';
      return a == 1000_b;
    });
    TEST([]()->ch_logic {
      ch_bit4 a;
      ch_bit4 b = a;     
      a.slice<3>(0) = '0';
      a[3] = '1';
      return a == 1000_b;
    });
    TEST([]()->ch_logic {
      ch_bit4 a;
      ch_bit4 b = a;
      a.slice<3>(0) = '0';
      ch_logic x = a[3];
      x = '1';
      return a == 1000_b;
    });
    TEST([]()->ch_logic {         
      ch_bit4 a, b(1);
      ch_bit4 c = a + b;
      ch_bit2 x = a.slice<2>(0);
      ch_bit2 y = a.slice<2>(1);
      ch_bit2 z = a.slice<2>(2);
      x[1] = 1; // x[1] and a[1] should change
      x = 0;    // x[0] and a[0] should change    
      z = 1;    // z and a[3] should change
      y = 0;    // y only should change
      return (c == 0111_b);
    });
    TEST([]()->ch_logic {
     ch_bit4 a;
     ch_logic x = a[3];
     ch_logic x2 = a[3];
     ch_bit2  y = a.slice<2>(2);
     ch_logic z = y[0];
     x = 1_b;
     z = 0_b;
     return (a[2] == 0_b && a[3] == 1_b);
    });
    TEST([]()->ch_logic {
     ch_bit4 a;
     ch_logic x = a[0];
     ch_logic y = a[1];     
     ch_bitv<3> z = a.slice<3>(0);
     ch_logic w = a[3];
     x = 0_b;
     y = 1_b;     
     z = 001_b;
     w = 1_b;
     return (a == 1010_b);
    });
  }
  
  SECTION("test concat", "[concat]") {
    TEST([]()->ch_logic {
      ch_bit4 a(1100_b);
      ch_bitv<5> c = (a, '1');
      return (c == 11001_b);
    });
    TEST([]()->ch_logic {
      ch_bit4 a(1100_b);
      ch_bitv<5> c = ('1', a);
      return (c == 11100_b);
    });
    TEST([]()->ch_logic {
      ch_bitv<6> c = (011_b, 011_b);
      return (c == 011011_b);
    });
    TEST([]()->ch_logic {
      ch_bitv<6> c = (10_b, 1011_b);
      return (c == 101011_b);
    });
    TEST([]()->ch_logic {
      ch_bit4 a(1100_b);
      ch_bit4 c = (a[3], a[2], a[1], a[0]);
      return (c == 1100_b);
    });
    TEST([]()->ch_logic {
       ch_bit4 a(1100_b);
       ch_bit4 c = (a[3], a[0], a[0], a[0]);
       return (c == 1000_b);
    });
  } 
  
  SECTION("test logic", "[logic]") {
    TEST([]()->ch_logic {
      ch_bit4 a(1100_b);
      ch_bit4 c = a & 0101_b;
      return (c == 0100_b);
    });
    TEST([]()->ch_logic {
      ch_bit4 a(1100_b);
      ch_bit4 c = a | 0101_b;
      return (c == 1101_b);
    });
    TEST([]()->ch_logic {
      ch_bit4 a(1100_b);
      ch_bit4 c = a ^ 0101_b;
      return (c == 1001_b);
    });
  }
  
  SECTION("test compare", "[compare]") {
    TEST([]()->ch_logic {
      return (0100_b == 0100_b);
    });
    TEST([]()->ch_logic {
      return (0101_b != 0100_b);
    });
    TEST([]()->ch_logic {
      return (0001_b < 0100_b);
    });
    TEST([]()->ch_logic {
      return (0100_b > 0010_b);
    });
    TEST([]()->ch_logic {
      return (0100_b <= 1100_b);
    });
    TEST([]()->ch_logic {
      return (0100_b >= 0100_b);
    });
    TEST([]()->ch_logic {
      return (1100_b >= 0100_b);
    });
  }
  
  SECTION("test shift", "[shift]") {
    TEST([]()->ch_logic {
      ch_bit4 a(1);
      ch_bit4 c = a << 2;
      return (c == 0100_b);
    });
    TEST([]()->ch_logic {
      ch_bit4 a(1000_b);
      ch_bit4 c = a >> 2;
      return (c == 0010_b);
    });
    TEST([]()->ch_logic {
      ch_bit4 a(1), b(2);
      ch_bit4 c = a << b;
      return (c == 0100_b);
    });
    TEST([]()->ch_logic {
      ch_bit4 a(1000_b), b(2);
      ch_bit4 c = a >> b;
      return (c == 0010_b);
    });
    TEST([]()->ch_logic {
      ch_bitv<64> a(100000000e64_h);
      ch_bitv<64> c = a >> 32;
      return (c == 1e64_h);
    });
    TEST([]()->ch_logic {
      ch_bitv<64> a(1);
      ch_bitv<64> c = a << 32;
      return (c == 100000000e64_h);
    });
    /*TEST([]()->ch_logic {
      ch_bit4 a(1000_b), b(2);
      ch_bit4 c = ch_rotl(a, b);
      return (c == 0010_b);
    });
    TEST([]()->ch_logic {
      ch_bit4 a(1000_b), b(2);
      ch_bit4 c = ch_rotr(a, b);
      return (c == 0010_b);
    });*/
  }
  
  SECTION("test arithmetic", "[math]") {
    TEST([]()->ch_logic {
      ch_bit4 a(0x1), b(0x2);
      ch_bit4 c = a + b;
      return (c == 0x3);
    });
    TEST([]()->ch_logic {
      ch_bitv<64> a(0xffffffff), b(0x1);
      ch_bitv<64> c = a + b;
      return (c == 100000000e64_h);
    });      
  }
}


