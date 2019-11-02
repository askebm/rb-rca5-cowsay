add_executable(WilmMain WilmMain.cpp)
target_link_libraries(WilmMain ${LIBS})

add_executable(marbleDetection1 marbleDetection1.cpp)
target_link_libraries(marbleDetection1 ${LIBS} Vision)

add_executable(SimpleRobotControllerInstance SimpleRobotControllerInstance.cpp)
target_link_libraries(SimpleRobotControllerInstance ${LIBS} SimpleRobotControl ${FUZZYLITE_LIB})
