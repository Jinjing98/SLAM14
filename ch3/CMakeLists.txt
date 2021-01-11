project(ch3_project)
set(CMAKE_BUILD_TYPE "Debug")

include_directories("/usr/include/eigen3")


add_executable(eigenmatrix useEigen/eigenMatrix.cpp)
add_executable(carrat useEigen/carrat.cpp)
