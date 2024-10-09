#ifndef POTENTIALFIELD_HPP
#define POTENTIALFIELD_HPP

#include "Matrix.hpp"

#define FREE_SPACE 0
#define OBSTACLE 1
#define OBJECTIVE -1
#define PATH -4

template <typename T> class PotentialField {
private:
  Matrix<T> map_array;   // Array with informations about the ambient,
                         // obstacles, objective, unreachable places, ...
  Matrix<T> field_array; // Calculated field for input map array

  T min_epsilon = 1e-4; // Smallest field difference for iteration to stop
  T epsilon;            // Calculated field difference between iterations

  std::size_t max_iterations = 1e4; // Maximum iterations for field calculation
  std::size_t iterated = 0; // Iterations executed before stopping calculation

public:
  PotentialField(const Matrix<T> &input_matrix)
      : map_array(input_matrix), field_array(input_matrix) {}

  std::vector<std::pair<std::size_t, std::size_t>> make_index_list();

  void gauss_seidel(void);

  inline T
  calculate_cell_field(const std::pair<std::size_t, std::size_t> &index) {
    return (field_array[index.first - 1][index.second] +
            field_array[index.first + 1][index.second] +
            field_array[index.first][index.second - 1] +
            field_array[index.first][index.second + 1]) /
           4;
  }
  T operator[](const std::pair<std::size_t, std::size_t> &index);
  bool verify_out_of_bounds(const std::pair<std::size_t, std::size_t> &index);

  std::pair<T, T> get_gradient(std::size_t row, std::size_t col);
  void set_min_epsilon(T value);
  T get_min_epsilon(void);
  T get_epsilon(void);
  void set_max_iterations(std::size_t value);
  std::size_t get_max_iterations(void);
  std::size_t get_iterated(void);

  Matrix<T> copy_map(void);
  Matrix<T> copy_field(void);

  void print_map(std::string file_name);
  void print_field(std::string file_name);
};

#include "PotentialField.tpp" // To include template definitions

#endif // POTENTIALFIELD_HPP
