################################################################################
#
# evoflock cmake file
# CMakeLists.txt
# August 22, 2024
# cwr
#
# See "cmake build of flock" in Notes app
# See CMake Tutorial: https://cmake.org/cmake/help/latest/guide/tutorial/
#
################################################################################

cmake_minimum_required(VERSION 3.10)

# Project name and version.
project(evoflock VERSION 0.0)

# Specify which C++ standard
#     Had been using 17 (for std::any), now 20 (for std::ranges).
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Specify compiler optimization level
#     In Xcode I have been using the default: "-Os" meaning "Fastest, Smallest".
add_compile_options(-O3)


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# For future use, toward linking with Open3D:
#
# # Find the external GLEW and GLFW packages.
# find_package(GLFW3 REQUIRED)
# find_package(GLEW REQUIRED)
# 
# # In target_link_libraries(): important to use "glfw" instead of "GLFW::glfw"
# # as suggested by https://stackoverflow.com/q/68871506/1991373
# target_link_libraries(flock glfw GLEW::glew)
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


# Very ad hoc link to pre-release version from "main-devel"
# (via https://github.com/isl-org/Open3D/releases/tag/main-devel)
#
# find_package(Open3D REQUIRED PATHS
#              /Users/cwr/Documents/code/open3d-devel-darwin-arm64-0.18.0+c219d82)

# Link to my own local build-from-source of Open3D
find_package(Open3D REQUIRED)


# add_executable(Draw)
# target_sources(Draw PRIVATE Draw.cpp)
# target_link_libraries(Draw PRIVATE Open3D::Open3D)

# Add source files to the executable
add_executable(evoflock main.cpp)
target_link_libraries(evoflock PRIVATE Open3D::Open3D)
