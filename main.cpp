#include "Matrix.hpp"
#include <iostream>

int main() {
  Matrix<int> test(4, 3);
  test.fill(1);
  test.data[1][0] = 3;
  test.data[2][1] = 5;
  test.data[1][2] = 8;
  test.data[0][1] = 10;
  auto padded = test.pad();
  padded.print("padded.csv");
}
