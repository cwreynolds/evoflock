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


# TODO 20240903 this was the (broken) version before Sep 3:
#
# cmake_minimum_required(VERSION 3.10)
# 
# # Project name and version.
# project(evoflock VERSION 0.0)
# 
# # Specify which C++ standard
# #     Had been using 17 (for std::any), now 20 (for std::ranges).
# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED True)
# 
# # Specify compiler optimization level
# #     In Xcode I have been using the default: "-Os" meaning "Fastest, Smallest".
# add_compile_options(-O3)
# 
# # Link to my own local build-from-source of Open3D
# find_package(Open3D REQUIRED)
# 
# # Add source files to the executable
# add_executable(evoflock main.cpp)
# target_link_libraries(evoflock PRIVATE Open3D::Open3D)


# TODO 20240904 oops still need this to use "ranges" in c++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

################################################################################


# Code below started as a copy of
# https://github.com/isl-org/Open3D/blob/main/examples/cmake/open3d-cmake-find-package/CMakeLists.txt

# Two sections below (marked off with #~~~~) are for Windows only. I do not
# anticipate running evoflock on Windows but as soon as I delete them I will
# certainly find I need to run on Windows.



# On Ubuntu 20.04, get the latest CMake from https://apt.kitware.com/.
cmake_minimum_required(VERSION 3.24)

# project(Open3DCMakeFindPackage LANGUAGES C CXX)
project(evoflock LANGUAGES C CXX)


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# The options need to be the same as Open3D's default
# If Open3D is configured and built with custom options, you'll also need to
# specify the same custom options.
option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
if(STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



# Find installed Open3D, which exports Open3D::Open3D
find_package(Open3D REQUIRED)

# add_executable(Draw)
# target_sources(Draw PRIVATE Draw.cpp)
# target_link_libraries(Draw PRIVATE Open3D::Open3D)

add_executable(evoflock)
target_sources(evoflock PRIVATE main.cpp)
target_link_libraries(evoflock PRIVATE Open3D::Open3D)



#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# On Windows if BUILD_SHARED_LIBS is enabled, copy .dll files to the executable directory
if(WIN32)
    get_target_property(open3d_type Open3D::Open3D TYPE)
    if(open3d_type STREQUAL "SHARED_LIBRARY")
        set(copy_dlls "${CMAKE_INSTALL_PREFIX}/bin/tbb12$<$<CONFIG:Debug>:_debug>.dll" 
                      "${CMAKE_INSTALL_PREFIX}/bin/Open3D.dll")
    else() 
        set(copy_dlls "${CMAKE_INSTALL_PREFIX}/bin/tbb12$<$<CONFIG:Debug>:_debug>.dll") 
    endif()
    add_custom_command(TARGET Draw POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy ${copy_dlls}
                        ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
                        COMMENT "Copying Open3D DLLs to ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
endif()
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
