add_executable(main ${COMMON_SOURCES} ${THIRD_PARTY_SOURCES} main.cpp)
target_include_directories(main PRIVATE ${PROJECT_SOURCE_DIR}/inc)
target_include_directories(main PRIVATE ${PROJECT_SOURCE_DIR}/third-party)
target_link_libraries(main ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARY} Eigen3::Eigen plog::plog imgui_lib )
