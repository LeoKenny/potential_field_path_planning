# Minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Project name and version
project(PotentialFieldPathPlanning VERSION 1.0)

# Set C++ standard to C++11 or later (you can adjust this to your desired C++ version)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories (this points to the include folder where Matrix.hpp and Matrix.tpp are located)
include_directories(include)

# Add the executable (this compiles the main.cpp file)
add_executable(main main.cpp)

