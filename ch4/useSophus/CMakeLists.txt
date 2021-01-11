project(learning_sophus)
set(CMAKE_BUILD_TYPE "Debug")


 

find_package(Sophus REQUIRED )
message("Sophus dir ${Sophus_INCLUDE_DIRS}")
message("Sophus lib ${Sophus_LIBRARIES}")
add_executable(useSophus useSophus.cpp)
include_directories(${Sophus_INCLUDE_DIRS})
target_link_libraries(useSophus ${Sophus_LIBRARIES})
