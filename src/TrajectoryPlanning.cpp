#include <cmath>
#include <cstdlib>

#include <TrajectoryPlanning.hpp>

#define HALF_MOVEMENT_RANGE_ANGLE 180.0
#define FULL_MOVEMENT_RANGE_ANGLE 360.0

std::size_t TrajectoryPlanning::get_iterated() { return iterated; }
std::size_t TrajectoryPlanning::get_max_iterations() { return max_iterations; }
void TrajectoryPlanning::set_max_iterations(std::size_t value) {
  max_iterations = value;
}

void TrajectoryPlanning::plan_path(
    std::pair<std::size_t, std::size_t> start_position) {
  Command cmd, previous_cmd;
  bool run_flag = true;

  iterated = 0;
  cmd.id = start_position;

  for (iterated = 0; iterated < max_iterations; iterated++) {
    get_smooth_gradient(&cmd);
    std::cout << "Mag Gradient: " << cmd.mag_gradient
              << "\nAngle Gradient: " << cmd.angle_gradient << std::endl;
    get_velocity(cmd, previous_cmd);
    std::cout << "Mag Velocity: " << cmd.mag_velocity
              << "\nAngle Velocity: " << cmd.angle_velocity << std::endl;
  }
}

void TrajectoryPlanning::get_smooth_gradient(Command *cmd) {
  double gradient_x = 0;
  double gradient_y = 0;
  int iterated_cells = 0;
  std::pair<double, double> gradient = {0, 0};
  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      gradient = pf.get_gradient(cmd->id.first + j, cmd->id.second + i);
      gradient_x += gradient.first;
      gradient_y += gradient.second;
      iterated_cells++;
    }
  }
  cmd->gradient = std::pair<double, double>(gradient_x / iterated_cells,
                                            gradient_y / iterated_cells);
  cmd->angle_gradient =
      std::atan2(cmd->gradient.second, cmd->gradient.first) * 180.0 / M_PI;
  cmd->mag_gradient = std::sqrt(std::pow(cmd->gradient.second, 2) +
                                std::pow(cmd->gradient.first, 2));
}

void TrajectoryPlanning::get_velocity(Command &cmd, Command &previous_cmd) {
  double Ve;                // Estimated Velocity
  double Vf;                // Final Velocity
  double Vd;                // Diference between estimated and previous velocity
  double ph;                // Velocity Angle Variation
  double rho;               // Distance ratio from obstacle/objective
  double rho_obstacle = 1;  // Distance ratio from obstacle
  double rho_objective = 1; // Distance ratio from objective
  double min_obst_distance; // Smallest distance from obstacles
  double objective_distance; // Distance from goal

  // Get velocity angle from gradient
  cmd.angle_velocity = cmd.angle_gradient;

  // Angle variation between steps
  ph = cmd.angle_velocity - previous_cmd.angle_velocity;
  if (ph > HALF_MOVEMENT_RANGE_ANGLE)
    ph -= FULL_MOVEMENT_RANGE_ANGLE;
  if (ph < -HALF_MOVEMENT_RANGE_ANGLE)
    ph += FULL_MOVEMENT_RANGE_ANGLE;

  // Get smallest rho value for velocity calculation
  /*min_obst_distance = get_min_obst_distance();*/
  /*objective_distance = get_objective_distance();*/
  min_obst_distance = 10;
  objective_distance = 20;

  if (min_obst_distance < rmax_obstacle)
    rho_obstacle = min_obst_distance / rmax_obstacle;
  if (objective_distance < rmax_objective)
    rho_objective = objective_distance / rmax_objective;

  rho = rho_obstacle;
  if (rho_objective < rho_obstacle)
    rho = rho_objective;

  // Calculate Estimated Velocity
  Ve = Vmax * (alpha + (1 - alpha) *
                           ((HALF_MOVEMENT_RANGE_ANGLE - std::abs(ph)) /
                            HALF_MOVEMENT_RANGE_ANGLE) *
                           rho);

  // Calculate Final Velocity
  Vd = Ve - previous_cmd.mag_velocity;
  if (std::abs(Vd) > eta * Vmax)
    Vf = previous_cmd.mag_velocity + ((eta * Vmax) * (Vd / std::abs(Vd)));
  else
    Vf = Ve;

  if (Vf > Vmax)
    cmd.mag_velocity = Vmax;
  else
    cmd.mag_velocity = Vf;
}
