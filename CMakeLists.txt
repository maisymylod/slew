# Canonical build. Requires only CMake and a C++20 compiler; nothing is
# fetched from the network. (A plain Makefile is also provided for machines
# without CMake.)
cmake_minimum_required(VERSION 3.16)
project(slew LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(SLEW_WERROR "Treat warnings as errors" ON)
option(SLEW_SANITIZE "Build with AddressSanitizer + UBSan" OFF)

add_library(slew_warnings INTERFACE)
target_compile_options(slew_warnings INTERFACE -Wall -Wextra -Wpedantic)
if(SLEW_WERROR)
  target_compile_options(slew_warnings INTERFACE -Werror)
endif()
if(SLEW_SANITIZE)
  target_compile_options(slew_warnings INTERFACE
    -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(slew_warnings INTERFACE -fsanitize=address,undefined)
endif()

# Core simulation library (plant, controller, integrator, loop).
add_library(slew_core src/simulation.cpp)
target_include_directories(slew_core PUBLIC include)
target_link_libraries(slew_core PUBLIC slew_warnings)
find_package(Threads REQUIRED)
target_link_libraries(slew_core PUBLIC Threads::Threads)

# Command-line driver.
add_executable(slew src/main.cpp)
target_link_libraries(slew PRIVATE slew_core)

# Test suite (dependency-free framework; one binary, self-registering cases).
enable_testing()
file(GLOB SLEW_TEST_SOURCES CONFIGURE_DEPENDS test/test_*.cpp)
add_executable(slew_tests ${SLEW_TEST_SOURCES})
target_include_directories(slew_tests PRIVATE test)
target_link_libraries(slew_tests PRIVATE slew_core)
add_test(NAME slew_tests COMMAND slew_tests)
