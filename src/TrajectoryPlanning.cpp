#include <TrajectoryPlanning.hpp>

#define HALF_MOVEMENT_RANGE_ANGLE 180.0
#define FULL_MOVEMENT_RANGE_ANGLE 360.0

std::size_t TrajectoryPlanning::get_iterated() { return iterated; }
std::size_t TrajectoryPlanning::get_max_iterations() { return max_iterations; }
void TrajectoryPlanning::set_max_iterations(std::size_t value) {
  max_iterations = value;
}

void TrajectoryPlanning::plan_trajectory(
    std::pair<double, double> start_position) {
  Command cmd, previous_cmd;

  cmd_list.clear();
  iterated = 0;
  cmd.position = start_position;
  cmd.grid_position =
      std::make_pair((std::size_t)(start_position.first / resolution),
                     (std::size_t)(start_position.second / resolution));

  for (iterated = 0;
       (iterated < max_iterations) &&
       (pf.verify_out_of_bounds(cmd.grid_position) == false) &&
       (get_min_obst_distance(cmd.grid_position) > colision_distance) &&
       (get_objective_distance(cmd.grid_position) > objective_distance);
       iterated++) {
    get_smooth_gradient(&cmd);
    get_velocity(cmd, previous_cmd);
    cmd_list.push_back(cmd);
    previous_cmd = cmd;
    cmd = get_next_position(cmd);
  }
  if (pf.verify_out_of_bounds(cmd.grid_position))
    std::cerr << "Out of bounds." << std::endl;
  if (iterated >= max_iterations)
    std::cerr << "Maximum iterations executed." << std::endl;
  if (get_min_obst_distance(cmd.grid_position) <= colision_distance)
    std::cerr << "Colision detected: "
              << get_min_obst_distance(cmd.grid_position) << std::endl;
  if (get_objective_distance(cmd.grid_position) <= objective_distance)
    std::cerr << "Objective reached." << std::endl;
}

Command TrajectoryPlanning::get_next_position(const Command &cmd) {
  Command next;
  next.position.first =
      std::sin(M_PI * cmd.angle_velocity / 180) * robot_movement_step;
  next.position.second =
      std::cos(M_PI * cmd.angle_velocity / 180) * robot_movement_step;
  next.position.first += cmd.position.first;
  next.position.second += cmd.position.second;
  next.grid_position.first = (std::size_t)(next.position.first / resolution);
  next.grid_position.second = (std::size_t)(next.position.second / resolution);
  return next;
}

void TrajectoryPlanning::get_smooth_gradient(Command *cmd) {
  double gradient_x = 0;
  double gradient_y = 0;
  int iterated_cells = 0;
  std::pair<double, double> gradient = {0, 0};
  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      gradient = pf.get_gradient(cmd->grid_position.first + j,
                                 cmd->grid_position.second + i);
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

double TrajectoryPlanning::get_min_obst_distance(
    const std::pair<std::size_t, std::size_t> &position) {
  double distance;
  double smallest_distance = std::sqrt(2 * std::pow(rmax_obstacle, 2));
  long int limits = rmax_obstacle / resolution;
  std::pair<std::size_t, std::size_t> grid_position = {0, 0};

  for (long i = -limits; i <= limits; i++) {
    for (long j = -limits; j <= limits; j++) {
      if (-(long long)position.first > i)
        continue;
      if (-(long long)position.second > j)
        continue;
      grid_position.first = position.first + i;
      grid_position.second = position.second + j;
      if (pf.verify_out_of_bounds(grid_position))
        continue;
      if (pf[grid_position] == OBSTACLE) {
        distance = std::sqrt(std::pow(i * resolution, 2) +
                             std::pow(j * resolution, 2));
        if (distance < smallest_distance)
          smallest_distance = distance;
      }
    }
  }

  return smallest_distance;
}

double TrajectoryPlanning::get_objective_distance(
    const std::pair<std::size_t, std::size_t> &position) {
  double distance;
  double smallest_distance = std::sqrt(2 * std::pow(rmax_objective, 2));
  long int limits = rmax_objective / resolution;
  std::pair<std::size_t, std::size_t> grid_position = {0, 0};

  for (long i = -limits; i <= limits; i++) {
    for (long j = -limits; j <= limits; j++) {
      if (-(long long)position.first > i)
        continue;
      if (-(long long)position.second > j)
        continue;
      grid_position.first = position.first + i;
      grid_position.second = position.second + j;
      if (pf.verify_out_of_bounds(grid_position))
        continue;
      if (pf[grid_position] == OBJECTIVE) {
        distance = std::sqrt(std::pow(i * resolution, 2) +
                             std::pow(j * resolution, 2));
        if (distance < smallest_distance)
          smallest_distance = distance;
      }
    }
  }

  return smallest_distance;
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
  min_obst_distance = get_min_obst_distance(cmd.grid_position);
  objective_distance = get_objective_distance(cmd.grid_position);

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
  cmd.Ve = Ve;

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

void TrajectoryPlanning::print_data(std::string file_name) {
  std::ofstream file(file_name + "_data.csv");
  if (file.is_open()) {
    file << std::fixed << std::setprecision(8);
    file << "PosX,PosY,Col,Row,Grad_Mag,Grad_Angle,Vel_Mag,Vel_Angle,Ve"
         << std::endl;
    for (auto &cmd : cmd_list) {
      file << cmd.position.second << "," << cmd.position.first << ","
           << cmd.grid_position.second << "," << cmd.grid_position.first << ","
           << cmd.mag_gradient << "," << cmd.angle_gradient << ","
           << cmd.mag_velocity << "," << cmd.angle_velocity << "," << cmd.Ve
           << std::endl;
    }
  } else
    std::cerr << "Failed to open file: " << file_name << std::endl;
}

void TrajectoryPlanning::print_field(std::string file_name) {
  pf.print_field(file_name + "_field.csv");
}

void TrajectoryPlanning::print_field_with_path(std::string file_name) {
  auto field_map = pf.copy_field();
  for (auto &cmd : cmd_list) {
    field_map[cmd.grid_position] = PATH;
  }
  field_map.print(file_name + "_field_w_path.csv");
}

void TrajectoryPlanning::print_map(std::string file_name) {
  pf.print_map(file_name + "_map.csv");
}

void TrajectoryPlanning::print_map_with_path(std::string file_name) {
  auto map = pf.copy_map();
  for (auto &cmd : cmd_list) {
    map[cmd.grid_position] = PATH;
  }
  map.print(file_name + "_map_w_path.csv");
}

std::vector<Command> TrajectoryPlanning::get_path() { return cmd_list; }
