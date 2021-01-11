set(CMAKE_BUILD_TYPE "Debug")
project(opencv_proj)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(imagebasics imageBasics.cpp)
target_link_libraries(imagebasics ${OpenCV_LIBS})
