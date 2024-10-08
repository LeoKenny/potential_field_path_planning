#ifndef TRAJECTORYPLANNING_HPP
#define TRAJECTORYPLANNING_HPP

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <ostream>
#include <utility>
#include <vector>

#include <Matrix.hpp>
#include <PotentialField.hpp>

constexpr std::pair<std::size_t, std::size_t> NO_NODE(-1, -1);

struct Command {
  std::pair<std::size_t, std::size_t> grid_position = NO_NODE;
  std::pair<double, double> position = {0, 0};
  std::pair<double, double> gradient = {0, 0};
  double mag_gradient = 0;
  double angle_gradient = 0;
  double mag_velocity = 0;
  double angle_velocity = 0;

  float residual_x = 0;
  float residual_y = 0;
  double Ve = 0;
  double Vmax = 0;
  float ph = 0;
};

class TrajectoryPlanning {
private:
  std::vector<Command> path;  // Path composed from commands
  PotentialField<double> pf;  // Potential Field element used for path planning
  std::size_t max_iterations; // Maximum number of iterations/steps
  std::size_t iterated;       // Number of iterations used in the planned path
  double resolution;          // Map resolution [m]
  double rmax_obstacle;       // Distance used for rho obstacle [m]
  double rmax_objective;      // Distance used for rho goal [m]
  double alpha;               // Percentage of the max velocity that is fixed
  double eta;                 // Maximum percentage of speed variation per step
  double Vmax;                // Maximum speed [m/s]
  double robot_movement_step; // Robot movement between steps [m]

  float min_goal_distance = 2;
  float residual_x = 0;
  float colision_distance = 8;
  float residual_y = 0;

public:
  TrajectoryPlanning(const PotentialField<double> &input, double resolution,
                     std::size_t max_iterations, double rmax_obstacle,
                     double rmax_objective, double alpha, double eta,
                     double Vmax, double robot_movement_step)
      : pf(input), max_iterations(max_iterations), resolution(resolution),
        rmax_obstacle(rmax_obstacle), rmax_objective(rmax_objective),
        alpha(alpha), eta(eta), Vmax(Vmax),
        robot_movement_step(robot_movement_step) {}

  std::size_t get_iterated();
  std::size_t get_max_iterations();
  void set_max_iterations(std::size_t value);

  void get_smooth_gradient(Command *cmd);
  void get_velocity(Command &cmd, Command &previous_cmd);
  double
  get_min_obst_distance(const std::pair<std::size_t, std::size_t> &position);
  double
  get_objective_distance(const std::pair<std::size_t, std::size_t> &position);
  Command get_next_position(const Command &cmd);

  void plan_path(std::pair<double, double> start_position);
};

#endif // TRAJECTORYPLANNING_HPP
