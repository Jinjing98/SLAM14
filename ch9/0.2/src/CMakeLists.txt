add_library( myslam SHARED
    
    camera.cpp
    config.cpp
    mappoint.cpp
    frame.cpp
    map.cpp
    visual_odometry.cpp


 
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)
