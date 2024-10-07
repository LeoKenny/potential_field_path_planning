#ifndef TRAJECTORYPLANNING_HPP
#define TRAJECTORYPLANNING_HPP

#include <cmath>
#include <cstdio>
#include <ostream>
#include <utility>
#include <vector>

#include <Matrix.hpp>
#include <PotentialField.hpp>

constexpr std::pair<std::size_t, std::size_t> NO_NODE(-1, -1);

struct Command {
  std::pair<std::size_t, std::size_t> id = NO_NODE;
  std::pair<double, double> gradient = {0, 0};
  double mag_gradient = 0;
  double angle_gradient = 0;

  double mag_velocity = 0;
  double angle_velocity = 0;
  float residual_x = 0;
  float residual_y = 0;
  double Ve = 0;
  double Vmax = 0;
  float goal_distance = 0;
  float min_wall_distance = 0;
  float rho = 0;
  float rho_wall = 0;
  float rho_goal = 0;
  float ph = 0;
};

class TrajectoryPlanning {
private:
  std::vector<Command> path;
  PotentialField<double> pf;
  std::size_t max_iterations = 200;
  std::size_t iterated = 0;

  double rmax = 21;
  double alpha = 0.01;
  double eta = 0.1;
  double Vmax = 1.0;
  float min_goal_distance = 2;
  float rmax_goal = 40;
  float residual_x = 0;
  float colision_distance = 8;
  float residual_y = 0;

public:
  TrajectoryPlanning(const PotentialField<double> &input) : pf(input) {}

  std::size_t get_iterated();
  std::size_t get_max_iterations();
  void set_max_iterations(std::size_t value);

  void get_smooth_gradient(Command *cmd);

  void plan_path(std::pair<std::size_t, std::size_t> start_position);
};

#endif // TRAJECTORYPLANNING_HPP
