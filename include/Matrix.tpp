#include <cstddef>
template <typename T>
Matrix<T>::Matrix(const int rows, const int cols)
    : data(rows, std::vector<T>(cols, 0)) {}

template <typename T> std::vector<T> &Matrix<T>::operator[](const int index) {
  return data[index];
}

template <typename T>
std::vector<T> &Matrix<T>::operator[](const std::size_t index) {
  return data[index];
}

template <typename T> std::vector<T> &Matrix<T>::operator[](const float index) {
  return data[int(index)];
}

template <typename T>
std::vector<T> &Matrix<T>::operator[](const std::pair<int, int> &index) {
  return data[index.first][index.second];
}

template <typename T>
std::vector<T> &Matrix<T>::operator[](const std::pair<float, float> &index) {
  return data[int(index.first)][int(index.second)];
}

template <typename T> const int Matrix<T>::getCols() const {
  return data[0].size();
}

template <typename T> const int Matrix<T>::getRows() const {
  return data.size();
}

template <typename T> void Matrix<T>::fill(T value) {
  const int rows = getRows();
  const int cols = getCols();
  for (std::size_t i = 0; i < rows; i++) {
    for (std::size_t j = 0; j < cols; j++) {
      data[i][j] = value;
    }
  }
}

template <typename T> Matrix<T> Matrix<T>::pad_with_value(T value) {
  const int rows = getRows();
  const int cols = getCols();

  Matrix<T> padded(rows + 2, cols + 2);

  for (std::size_t i = 0; i < rows; i++) {
    for (std::size_t j = 0; j < cols; j++) {
      padded[i + 1][j + 1] = data[i][j];
    }
  }

  std::fill(padded[0].begin(), padded[0].end(), value);
  std::fill(padded[rows + 1].begin(), padded[rows + 1].end(), value);

  for (std::size_t i = 0; i < padded.getRows(); i++) {
    padded[i][0] = value;
    padded[i][cols + 1] = value;
  }

  return padded;
}

template <typename T> Matrix<T> Matrix<T>::pad() {
  const int rows = getRows();
  const int cols = getCols();

  Matrix<T> padded(rows + 2, cols + 2);

  for (std::size_t i = 0; i < rows; i++) {
    std::copy(data[i].begin(), data[i].end(), padded[i + 1].begin() + 1);
  }

  std::copy(data[0].begin(), data[0].end(), padded[0].begin() + 1);
  std::copy(data[rows - 1].begin(), data[rows - 1].end(),
            padded[rows + 1].begin() + 1);

  for (std::size_t i = 0; i < padded.getRows(); i++) {
    padded[i][0] = padded[i][1];
    padded[i][cols + 1] = padded[i][cols];
  }

  return padded;
}

template <typename T> void Matrix<T>::print(const std::string file_name) const {
  std::ofstream file(file_name);
  if (file.is_open()) {
    file << std::fixed << std::setprecision(std::numeric_limits<T>::digits10);
    const int rows = getRows();
    const int cols = getCols();
    for (std::size_t i = 0; i < rows; i++) {
      for (std::size_t j = 0; j < cols; j++) {
        file << data[i][j] << ",\t";
      }
      file << std::endl;
    }
  } else {
    std::cerr << "Unable to open file: " << file_name << std::endl;
  }
}
