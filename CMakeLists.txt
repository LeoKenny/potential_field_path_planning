# Minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Project name and version
project(PotentialFieldPathPlanning VERSION 1.0)

# Set C++ standard to C++11 or later (you can adjust this to your desired C++ version)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set build configurations
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Include directories (this points to the include folder where Matrix.hpp and Matrix.tpp are located)
set(HEADER_DIR include)
set(HEADER_FILES ${HEADER_DIR}/Matrix.hpp
		 ${HEADER_DIR}/Matrix.tpp
		 ${HEADER_DIR}/PotentialField.hpp)

add_library(potentialfield src/PotentialField.cpp ${HEADER_FILES})
target_include_directories(potentialfield PRIVATE ${HEADER_DIR})

# Add the executable (this compiles the main.cpp file)
add_executable(main src/main.cpp src/PotentialField.cpp ${HEADER_FILES})
target_include_directories(main PRIVATE ${HEADER_DIR})

