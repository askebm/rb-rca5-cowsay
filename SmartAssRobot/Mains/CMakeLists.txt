add_executable(WilmMain WilmMain.cpp)
target_link_libraries(WilmMain ${LIBS})

add_executable(marbleDetection1 marbleDetection1.cpp)
target_link_libraries(marbleDetection1 ${LIBS} Vision)

add_executable(SimpleRobotControllerInstance SimpleRobotControllerInstance.cpp)
target_link_libraries(SimpleRobotControllerInstance ${LIBS} SimpleRobotControl ${FUZZYLITE_LIB})

add_executable(QLearner QLearner.cpp)
target_link_libraries(QLearner ${LIBS} QAgent)

add_executable(QL_Circle_test QL_circle_test.cpp)
target_link_libraries(QL_Circle_test ${LIBS} QAgent)

add_executable(Matlab Matlab_tests.cpp)
target_link_libraries(Matlab ${LIBS} QAgent)

add_executable(FuzzyAtivationRate FuzzyControlActivationRate.cpp)
target_link_libraries(FuzzyAtivationRate ${LIBS} SimpleRobotControl ${FUZZYLITE_LIB})
