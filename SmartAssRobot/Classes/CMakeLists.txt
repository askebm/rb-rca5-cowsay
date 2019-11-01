add_library(Vision ${CMAKE_CURRENT_SOURCE_DIR}/Camera/Vision.cpp)
#install	(TARGETS Vision DESTINATION bin)
#install (FILES ${CMAKE_CURRENT_SOURCE_DIR}/Camera/Vision.h DESTINATION include)
add_library(path_planning ${CMAKE_CURRENT_SOURCE_DIR}/Lidar/path_planning.cpp)
