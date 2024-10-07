#include <cmath>
#include <cstdlib>

#include <TrajectoryPlanning.hpp>

std::size_t TrajectoryPlanning::get_iterated() { return iterated; }
std::size_t TrajectoryPlanning::get_max_iterations() { return max_iterations; }
void TrajectoryPlanning::set_max_iterations(std::size_t value) {
  max_iterations = value;
}

void TrajectoryPlanning::plan_path(
    std::pair<std::size_t, std::size_t> start_position) {
  Command next;
  bool run_flag = true;

  iterated = 0;
  next.id = start_position;
  get_smooth_gradient(&next);
  std::cout << "Mag Gradient: " << next.mag_gradient
            << "\nAngle Gradient: " << next.angle_gradient << std::endl;

  /*for (iterated = 0; iterated < max_iterations; iterated++) {*/
  /*}*/
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
