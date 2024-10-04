#ifndef POTENTIALFIELD_HPP
#define POTENTIALFIELD_HPP

// the configured options and settings for Tutorial
#include "Matrix.hpp"

class PotentialField {
private:
  Matrix<double> map_array;
  Matrix<double> field_array;
  double eps;

public:
  PotentialField(const Matrix<double> &input_matrix)
      : map_array(input_matrix), field_array(input_matrix) {}

  std::vector<std::pair<std::size_t, std::size_t>> make_index_list();
  void print_map(void);
};

#endif // POTENTIALFIELD_HPP
