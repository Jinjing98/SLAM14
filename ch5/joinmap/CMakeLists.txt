project(joinmap)
set(CMAKE_BUILD_TYPE "Debug")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

find_package(PCL REQUIRED COMPONENT common io)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINATIONS})


add_executable(joinmap joinmap.cpp)
target_link_libraries(joinmap ${OpenCV_LIBS} ${PCL_LIBRARIES})

