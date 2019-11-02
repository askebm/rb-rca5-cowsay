add_library(Vision ${CMAKE_CURRENT_SOURCE_DIR}/Camera/Vision.cpp)
#install	(TARGETS Vision DESTINATION bin)
#install (FILES ${CMAKE_CURRENT_SOURCE_DIR}/Camera/Vision.h DESTINATION include)

execute_process(COMMAND fuzzylite -i SimpleRobotControllerEngine.fll -of cpp
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Robot
	OUTPUT_VARIABLE FUZZYENGINE)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Robot/fuzzyEngine.cpp.in
	${CMAKE_CURRENT_SOURCE_DIR}/Robot/fuzzyEngine.cpp)

add_library(SimpleRobotControl ${CMAKE_CURRENT_SOURCE_DIR}/Robot/SimpleRobotControl.cpp)
