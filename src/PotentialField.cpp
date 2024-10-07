#include <PotentialField.hpp>

void PotentialField::print_map(void) { map_array.print("map_array.csv"); }
void PotentialField::print_field(void) { field_array.print("field_array.csv"); }

std::vector<std::pair<std::size_t, std::size_t>>
PotentialField::make_index_list() {
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

void PotentialField::gauss_seidel() {
  epsilon = std::numeric_limits<double>::max();

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
