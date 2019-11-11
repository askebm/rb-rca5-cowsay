add_executable(WilmMain WilmMain.cpp)
target_link_libraries(WilmMain ${LIBS})

add_executable(marbleDetection1 marbleDetection1.cpp)
target_link_libraries(marbleDetection1 ${LIBS} Vision)

add_executable(localization brushfire_test.cpp)
target_link_libraries(localization ${LIBS} path_planning)
