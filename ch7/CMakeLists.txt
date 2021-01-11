project(char7)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")
list(APPEND CMAKE_MODULE_PATH /home/jinjing/package/g2o/cmake_modules )
 

find_package(OpenCV 3 REQUIRED)
find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)
 

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)




add_executable(2d2d 2d2d.cpp)
target_link_libraries(2d2d
        g2o_core g2o_stuff
        ${OpenCV_LIBS})

        
add_executable(featurepoints featurepoints.cpp)
target_link_libraries(featurepoints ${OpenCV_LIBS})

add_executable(triangulation triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBS})

        
add_executable(3d2d 3d2d.cpp)
target_link_libraries(3d2d
        g2o_core g2o_stuff 
        ${OpenCV_LIBS} ${Sophus_LIBRARIES})
        
add_executable(3d3d  3d3d.cpp)
target_link_libraries(3d3d
        g2o_core g2o_stuff
        ${OpenCV_LIBS} ${Sophus_LIBRARIES})
