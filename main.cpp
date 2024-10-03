#include "Matrix.hpp"
#include <iostream>

int main() {
  Matrix<int> test(4, 3);
  test.fill(0);
  test.data[1][0] = 3;
  test.data[2][1] = 5;
  test.data[1][2] = 8;
  test.data[0][1] = 10;
  test.print("test.csv");

  auto padded = test.pad();
  padded.print("padded.csv");

  std::pair<std::size_t, std::size_t> start;
  start.first = 4;
  start.second = 3;
  std::pair<std::size_t, std::size_t> end;
  end.first = 1;
  end.second = 1;
  auto sliced = padded.slice(start, end);
  sliced.print("sliced.csv");

  auto test2 = sliced.copy();
  test2.print("test2.csv");

  Matrix<int> test3(4, 3);
  test3.fill(1);
  test3.copy_non_zero_values(test2);
  test3.print("test3.csv");
}
