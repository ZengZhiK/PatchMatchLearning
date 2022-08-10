cmake_minimum_required(VERSION 3.22)
project(PatchMatchLearning)

set(CMAKE_CXX_STANDARD 14)

set(OpenCV_DIR "D:\\DevTool\\OpenCV\\opencv-3.4.16\\mingw-build")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(PatchMatchLearning main.cpp PatchMatchStereo.cpp PatchMatchStereo.h CostComputer.hpp PMSPropagation.cpp PMSPropagation.h)

target_link_libraries(PatchMatchLearning ${OpenCV_LIBS})
