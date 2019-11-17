add_library(Vision ${CMAKE_CURRENT_SOURCE_DIR}/Camera/Vision.cpp)
#install	(TARGETS Vision DESTINATION bin)
#install (FILES ${CMAKE_CURRENT_SOURCE_DIR}/Camera/Vision.h DESTINATION include)

execute_process(COMMAND fuzzylite -i SimpleRobotControllerEngine.fll -of cpp
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Robot
	OUTPUT_VARIABLE FUZZYENGINE)

string(REGEX REPLACE "(using namespace fl\n)" "\\1Engine* rba::CreateEngine\(\) {\n" FUZZYENGINE ${FUZZYENGINE})
string(REGEX REPLACE "([A-Za-z)])\n" "\\1;\n" FUZZYENGINE ${FUZZYENGINE})

#set(FUZZYENGINE "${FUZZY_ENGINE_SANITIZED}")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Agent)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Robot/fuzzyEngine.cpp.in
	${CMAKE_CURRENT_SOURCE_DIR}/Robot/fuzzyEngine.cpp)

add_library(SimpleRobotControl ${CMAKE_CURRENT_SOURCE_DIR}/Robot/SimpleRobotControl.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Robot/fuzzyEngine.cpp)
