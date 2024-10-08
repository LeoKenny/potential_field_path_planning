#include "Matrix.hpp"
#include "PotentialField.hpp"
#include "TrajectoryPlanning.hpp"

#include <cstddef>
#include <iostream>
#include <utility>

int main() {
  Matrix<int> test(4, 3);
  test.fill(0);
  test[1][0] = 3;
  test[2][1] = 5;
  test[1][2] = 8;
  test[0][1] = 10;
  test.print("test.csv");

  auto padded = test.pad();
  padded.print("padded.csv");

  std::pair<std::size_t, std::size_t> start;
  start.first = 4;
  start.second = 3;
  std::pair<std::size_t, std::size_t> end;
  end.first = 1;
  end.second = 1;
  std::pair<std::size_t, std::size_t> other;
  other.first = 2;
  other.second = 3;

  std::cout << "Taking Value Start: " << padded[start] << std::endl;
  std::cout << "Taking Value End: " << padded[end] << std::endl;
  std::cout << "Taking Value Other: " << padded[other] << std::endl;
  padded[other] = 99;
  std::cout << "Taking Value New Other: " << padded[other] << std::endl;

  auto sliced = padded.slice(start, end);
  sliced.print("sliced.csv");

  auto test2 = sliced.copy();
  test2.print("test2.csv");

  Matrix<int> test3(4, 3);
  test3.fill(1);
  test3.copy_non_zero_values(test2);
  test3.print("test3.csv");

  std::pair<int, int> pos1(1, 1);
  auto res1 = test3.verify_out_of_bounds(pos1);
  std::cout << "Verify out of bounds (" << pos1.first << "," << pos1.second
            << "): " << res1 << std::endl;

  std::pair<int, int> pos2(4, 2);
  auto res2 = test3.verify_out_of_bounds(pos2);
  std::cout << "Verify out of bounds (" << pos2.first << "," << pos2.second
            << "): " << res2 << std::endl;

  std::pair<int, int> pos3(3, 3);
  auto res3 = test3.verify_out_of_bounds(pos3);
  std::cout << "Verify out of bounds (" << pos3.first << "," << pos3.second
            << "): " << res3 << std::endl;

  Matrix<double> test4(100, 100);
  test4.fill(0);
  auto test5 = test4.pad_with_value(1);
  test5[50][50] = -1;
  PotentialField<double> pot(test5);
  pot.gauss_seidel();
  pot.print_map();
  pot.print_field();
  std::cout << "Iterations: " << pot.get_iterated()
            << "\nEpsilon: " << pot.get_epsilon() << std::endl;

  Matrix<float> test6(100, 100);
  test6.fill(0);
  auto test7 = test6.pad_with_value(1);
  test6[50][50] = -1;
  PotentialField<float> pot2(test6);
  pot2.gauss_seidel();
  std::cout << "Iterations: " << pot2.get_iterated()
            << "\nEpsilon: " << pot2.get_epsilon() << std::endl;

  std::size_t max_iterations = 4;
  double resolution = 0.1;
  double rmax_obstacle = 22;
  double rmax_objective = 40;
  double alpha = 0.01;
  double eta = 0.1;
  double Vmax = 1.0;
  double robot_movement_step = 1.0;

  TrajectoryPlanning tp(pot, resolution, max_iterations, rmax_obstacle,
                        rmax_objective, alpha, eta, Vmax, robot_movement_step);
  std::pair<double, double> robot_start;
  robot_start.first = 2;
  robot_start.second = 2;
  tp.plan_path(robot_start);
}
