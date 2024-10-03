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

template <typename T> const std::size_t Matrix<T>::getCols() const {
  return data[0].size();
}

template <typename T> const std::size_t Matrix<T>::getRows() const {
  return data.size();
}

template <typename T> void Matrix<T>::fill(T value) {
  const std::size_t rows = getRows();
  const std::size_t cols = getCols();
  for (std::size_t i = 0; i < rows; i++) {
    for (std::size_t j = 0; j < cols; j++) {
      data[i][j] = value;
    }
  }
}

template <typename T> Matrix<T> Matrix<T>::pad_with_value(T value) {
  const std::size_t rows = getRows();
  const std::size_t cols = getCols();

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
  const std::size_t rows = getRows();
  const std::size_t cols = getCols();

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

template <typename T>
Matrix<T> Matrix<T>::slice(std::pair<std::size_t, std::size_t> start,
                           std::pair<std::size_t, std::size_t> end) {
  std::size_t size_rows;
  std::size_t size_cols;
  std::size_t lower_limit_rows;
  std::size_t lower_limit_cols;

  if (start.first < 0)
    start.first = 0;
  if (start.second < 0)
    start.second = 0;

  if (start.first >= getRows())
    start.first = getRows() - 1;
  if (start.second >= getCols())
    start.second = getCols() - 1;

  if (end.first < 0)
    end.first = 0;
  if (end.second < 0)
    end.second = 0;

  if (end.first >= getRows())
    end.first = getRows() - 1;
  if (end.second >= getCols())
    end.second = getCols() - 1;

  std::cout << "Start (" << start.first << "," << start.second << ") \t End ("
            << end.first << "," << end.second << ")" << std::endl;

  if (end.first < start.first) {
    lower_limit_rows = end.first;
    size_rows = start.first - end.first;
  } else {
    lower_limit_rows = start.first;
    size_rows = end.first - start.first;
  }
  size_rows++;

  if (end.second < start.second) {
    lower_limit_cols = end.second;
    size_cols = start.second - end.second;
  } else {
    lower_limit_cols = start.second;
    size_cols = end.second - start.second;
  }
  size_cols++;

  std::cout << "Lower Limits: (" << lower_limit_rows << "," << lower_limit_cols
            << ") \t Size (" << size_rows << "," << size_cols << ")"
            << std::endl;

  Matrix<T> sliced(size_rows, size_cols);

  for (std::size_t i = 0; i < size_rows; i++) {
    /*for (std::size_t j = 0; j < size_cols; j++) {*/
    /*  sliced[i][j] = data[lower_limit_rows + i][lower_limit_cols + j];*/
    /*}*/
    std::copy(data[lower_limit_rows + i].begin() + lower_limit_cols,
              data[lower_limit_rows + i].begin() + lower_limit_cols + size_cols,
              sliced[i].begin());
  }

  return sliced;
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
