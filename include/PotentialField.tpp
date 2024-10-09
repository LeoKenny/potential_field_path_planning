#include <PotentialField.hpp>

template <typename T> void PotentialField<T>::print_map(std::string file_name) {
  map_array.print(file_name);
}
template <typename T>
void PotentialField<T>::print_field(std::string file_name) {
  field_array.print(file_name);
}

template <typename T> Matrix<T> PotentialField<T>::copy_map(void) {
  return map_array.copy();
}
template <typename T> Matrix<T> PotentialField<T>::copy_field(void) {
  return field_array.copy();
}

template <typename T>
T PotentialField<T>::operator[](
    const std::pair<std::size_t, std::size_t> &index) {
  return field_array[index.first][index.second];
}

template <typename T>
bool PotentialField<T>::verify_out_of_bounds(
    const std::pair<std::size_t, std::size_t> &index) {
  return field_array.verify_out_of_bounds(index);
}

template <typename T>
void PotentialField<T>::set_max_iterations(std::size_t value) {
  max_iterations = value;
}

template <typename T> std::size_t PotentialField<T>::get_max_iterations(void) {
  return max_iterations;
}

template <typename T> std::size_t PotentialField<T>::get_iterated(void) {
  return iterated;
}

template <typename T> T PotentialField<T>::get_min_epsilon(void) {
  return min_epsilon;
}

template <typename T> void PotentialField<T>::set_min_epsilon(T value) {
  min_epsilon = value;
}

template <typename T> T PotentialField<T>::get_epsilon(void) { return epsilon; }

template <typename T>
std::vector<std::pair<std::size_t, std::size_t>>
PotentialField<T>::make_index_list() {
  std::vector<std::pair<std::size_t, std::size_t>> index_list;
  std::size_t rows = map_array.getRows();
  std::size_t cols = map_array.getCols();

  // Left->Right, Down->Up
  for (size_t i = 1; i < rows - 1; i++) {
    for (size_t j = 1; j < cols - 1; j++) {
      if (map_array[i][j] == FREE_SPACE)
        index_list.push_back(std::make_pair(i, j));
    }
  }

  // Right->Left, Up->Down
  for (size_t i = rows - 2; i > 0; i--) {
    for (size_t j = cols - 2; j > 0; j--) {
      if (map_array[i][j] == FREE_SPACE)
        index_list.push_back(std::make_pair(i, j));
    }
  }

  // Right->Left, Down->Up
  for (size_t i = 1; i < rows - 1; i++) {
    for (size_t j = cols - 2; j > 0; j--) {
      if (map_array[i][j] == FREE_SPACE)
        index_list.push_back(std::make_pair(i, j));
    }
  }

  // Left->Right, Up->Down
  for (size_t i = rows - 2; i > 0; i--) {
    for (size_t j = 1; j < cols - 1; j++) {
      if (map_array[i][j] == FREE_SPACE)
        index_list.push_back(std::make_pair(i, j));
    }
  }

  return index_list;
}

template <typename T>
std::pair<T, T> PotentialField<T>::get_gradient(std::size_t row,
                                                std::size_t col) {
  T vector_x = field_array[row][col - 1] - field_array[row][col + 1];
  T vector_y = field_array[row - 1][col] - field_array[row + 1][col];
  return std::make_pair(vector_x, vector_y);
}

template <typename T> void PotentialField<T>::gauss_seidel(void) {
  epsilon = std::numeric_limits<T>::max();

  map_array = map_array.pad();

  std::vector<std::pair<std::size_t, std::size_t>> index_list =
      make_index_list();

  for (iterated = 0; (iterated < max_iterations) && (epsilon > min_epsilon);
       iterated++) {
    auto old_field_array = field_array.copy();
    field_array = field_array.pad();
    for (std::pair<std::size_t, std::size_t> index : index_list) {
      field_array[index] = calculate_cell_field(index);
    }

    for (std::pair<std::size_t, std::size_t> index : index_list) {
      field_array[index] = calculate_cell_field(index);
    }

    field_array = field_array.slice(
        std::make_pair(1, 1),
        std::make_pair(field_array.getRows() - 2, field_array.getCols() - 2));

    epsilon = field_array.calculate_max_difference(old_field_array);
  }

  map_array = map_array.slice(
      std::make_pair(1, 1),
      std::make_pair(map_array.getRows() - 2, map_array.getCols() - 2));
}
