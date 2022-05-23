#include "test-utils.h"
#include <iostream>

int main() {
    float8 k1 = 6.899999;
    float8 k2 = 5.1;
    float8 k4 = k1-k2;
    float k = float(k4);
    std::cout << (k) << std::endl;
    std::cout << k4;
}   