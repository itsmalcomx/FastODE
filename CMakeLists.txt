cmake_minimum_required(VERSION 3.18)
project(FastODE VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fix Python policy warning
cmake_policy(SET CMP0148 NEW)
cmake_policy(SET CMP0144 NEW)

# Tell CMake where Python is (Windows only)
if(WIN32)
    set(Python3_EXECUTABLE "C:/Program Files/Python314/python.exe")
    set(Python3_ROOT_DIR "C:/Program Files/Python314")
    set(GTEST_ROOT "C:/Program Files (x86)/googletest-distribution")
endif()

# Find pybind11
find_package(pybind11 REQUIRED)

# Core C++ library
add_library(fastode_core STATIC
    src/rk4.cpp
)
target_include_directories(fastode_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)
set_target_properties(fastode_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# Python bindings
pybind11_add_module(fastode
    python/bindings.cpp
)
target_link_libraries(fastode PRIVATE fastode_core)

# Tests
enable_testing()
find_package(GTest REQUIRED)

add_executable(test_rk4 tests/test_rk4.cpp)
target_link_libraries(test_rk4
    PRIVATE fastode_core GTest::GTest GTest::Main
)
add_test(NAME test_rk4 COMMAND test_rk4)