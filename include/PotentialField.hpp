#ifndef POTENTIALFIELD_HPP
#define POTENTIALFIELD_HPP

#include "Matrix.hpp"

#define FREE_SPACE 0
#define OBSTACLE 1
#define OBJECTIVE -1

class PotentialField {
private:
  Matrix<double> map_array;   // Array with informations about the ambient,
                              // obstacles, objective, unreachable places, ...
  Matrix<double> field_array; // Calculated field for input map array
  double min_epsilon = 1e-9;  // Smalles field difference for iteration stop
  std::size_t max_iterations = 1e4; // Maximum iterations for field calculation

public:
  double epsilon;       // Calculated field difference between iterations
  std::size_t iterated; // Iterations executed before stopping calculation

  PotentialField(const Matrix<double> &input_matrix)
      : map_array(input_matrix), field_array(input_matrix) {}

  std::vector<std::pair<std::size_t, std::size_t>> make_index_list();
  void print_map(void);
  void print_field(void);

  void gauss_seidel();

  inline double
  calculate_cell_field(const std::pair<std::size_t, std::size_t> &index) {
    return (field_array[index.first - 1][index.second] +
            field_array[index.first + 1][index.second] +
            field_array[index.first][index.second - 1] +
            field_array[index.first][index.second + 1]) /
           4;
  }
};

#endif // POTENTIALFIELD_HPP
