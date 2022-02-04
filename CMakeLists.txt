cmake_minimum_required(VERSION 3.5)
project(MR_test)
find_package (Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(glm REQUIRED)
find_package(glfw3 REQUIRED)

find_package(GLEW REQUIRED)


INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include/MR")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include/common")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
SET(MR include/MR/modern_robotics.cpp)
add_executable(main src/main.cpp ${MR} )
target_link_libraries(main ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLM_LIBRARY} ${GLFW3_LIBRARY} ${GLEW_LIBRARY})

