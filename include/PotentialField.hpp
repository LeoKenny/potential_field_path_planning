#ifndef POTENTIALFIELD_HPP
#define POTENTIALFIELD_HPP

#include "Matrix.hpp"

#define FREE_SPACE 0
#define OBSTACLE 1
#define OBJECTIVE -1

template <typename T> class PotentialField {
private:
  Matrix<T> map_array;   // Array with informations about the ambient,
                         // obstacles, objective, unreachable places, ...
  Matrix<T> field_array; // Calculated field for input map array
  T min_epsilon = 1e-4;  // Smalles field difference for iteration stop
  T epsilon;             // Calculated field difference between iterations
  std::size_t max_iterations = 1e4; // Maximum iterations for field calculation
  std::size_t iterated; // Iterations executed before stopping calculation

public:
  PotentialField(const Matrix<T> &input_matrix)
      : map_array(input_matrix), field_array(input_matrix) {}

  std::vector<std::pair<std::size_t, std::size_t>> make_index_list();
  void print_map(void);
  void print_field(void);

  void gauss_seidel(void);

  inline T
  calculate_cell_field(const std::pair<std::size_t, std::size_t> &index) {
    return (field_array[index.first - 1][index.second] +
            field_array[index.first + 1][index.second] +
            field_array[index.first][index.second - 1] +
            field_array[index.first][index.second + 1]) /
           4;
  }

  void set_min_epsilon(T value);
  T get_min_epsilon(void);
  T get_epsilon(void);
  void set_max_iterations(std::size_t value);
  std::size_t get_max_iterations(void);
  std::size_t get_iterated(void);
};

#include "PotentialField.tpp" // To include template definitions

#endif // POTENTIALFIELD_HPP
