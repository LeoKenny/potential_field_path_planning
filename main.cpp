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
  std::pair<std::size_t, std::size_t> start;
  start.first = 4;
  start.second = 3;
  std::pair<std::size_t, std::size_t> end;
  end.first = 1;
  end.second = 1;
  auto sliced = padded.slice(start, end);
  test.print("test.csv");
  padded.print("padded.csv");
  sliced.print("sliced.csv");
}
