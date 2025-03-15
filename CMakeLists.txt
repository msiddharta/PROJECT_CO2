# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly

# Ensure to set the target (for example, ESP32):
set(EXTRA_COMPONENT_DIRS ${CMAKE_SOURCE_DIR}/BirdieCo2)  # Include the BirdieCo2 component directory

cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(PROJECT_CO2)