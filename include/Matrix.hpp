#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

template <typename T> class Matrix {
private:
  const int goal_field = -1;
  const int obstacle_field = 1;
  const int free_space = 0;

public:
  std::vector<std::vector<T>> data;

  Matrix(const int rows, const int cols);

  std::vector<T> &operator[](const int index);
  std::vector<T> &operator[](const std::size_t index);
  std::vector<T> &operator[](const float index);
  std::vector<T> &operator[](const std::pair<int, int> &index);
  std::vector<T> &operator[](const std::pair<float, float> &index);

  const std::size_t getCols() const;
  const std::size_t getRows() const;

  void fill(T value);
  Matrix<T> copy() const;
  void copy_non_zero_values(Matrix &source);
  Matrix<T> pad();
  Matrix<T> pad_with_value(T value);
  Matrix<T> slice(std::pair<std::size_t, std::size_t> start,
                  std::pair<std::size_t, std::size_t> end);
  void print(const std::string file_name) const;
};

#include "Matrix.tpp" // To include template definitions

#endif // MATRIX_HPP
