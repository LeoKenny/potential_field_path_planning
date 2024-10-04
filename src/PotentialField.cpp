#include <PotentialField.hpp>

void PotentialField::print_map(void) { map_array.print("map_array.csv"); }

std::vector<std::pair<std::size_t, std::size_t>>
PotentialField::make_index_list() {
  std::size_t rows = map_array.getRows();
  std::size_t cols = map_array.getCols();
}
