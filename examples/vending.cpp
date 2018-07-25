#include <cash.h>

using namespace ch::core;
using namespace ch::sim;

__enum (Coin, (dime, nickel));
__enum (State, (idle, s5, s10, s15, sOk));

struct VendingMachine {
  __io (
    __in(Coin)     coin,
    __out(ch_bool) valid
  );

  void describe() {
    ch_reg<State> state(State::idle);
    __switch (state)
    __case (State::idle) {
      __if (io.coin == Coin::nickel) { state->next = State::s5; };
      __if (io.coin == Coin::dime) { state->next = State::s10; };
    }
    __case (State::s5) {
      __if (io.coin == Coin::nickel) { state->next = State::s10; };
      __if (io.coin == Coin::dime) { state->next = State::s15; };
    }
    __case (State::s10) {
      __if (io.coin == Coin::nickel) { state->next = State::s15; };
      __if (io.coin == Coin::dime) { state->next = State::sOk; };
    }
    __case (State::s15) {
      __if (io.coin == Coin::nickel) { state->next = State::sOk; };
      __if (io.coin == Coin::dime) { state->next = State::sOk; };
    }
    __case (State::sOk) {
      state->next = State::idle;
    };
    io.valid = (state == State::sOk);
  }
};

int main() {
  ch_device<VendingMachine> vending;

  vending.io.coin = Coin::nickel;

  ch_tracer tracer(vending);
  tracer.run(2*(1+4));

  std::cout << "result:" << std::endl;
  std::cout << "valid = "  << vending.io.valid << std::endl;

  assert(vending.io.valid);

  tracer.toVCD("vending.vcd");

  ch_toVerilog("vending.v", vending);
  ch_toFirrtl("vending.fir", vending);

  int ret = system("iverilog vending_tb.v -o vending_tb.iv")
          | system("! vvp vending_tb.iv | grep 'ERROR' || false");
  std::cout << "ret=" << ret << std::endl;
  return ret != 0;
}
