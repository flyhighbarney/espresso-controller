# Host-side unit tests — compile with native GCC, no ARM cross-compiler needed
cmake_minimum_required(VERSION 3.22)
project(espresso-tests C)

set(CMAKE_C_STANDARD 11)
add_definitions(-DUNIT_TEST)

# Link math library
link_libraries(m)

add_executable(test_pid test_pid.c)
add_executable(test_kalman test_kalman.c)
add_executable(test_cobs test_cobs.c)

enable_testing()
add_test(NAME pid_tests COMMAND test_pid)
add_test(NAME kalman_tests COMMAND test_kalman)
add_test(NAME cobs_tests COMMAND test_cobs)
