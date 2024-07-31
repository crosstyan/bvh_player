cmake_minimum_required(VERSION 3.20)
project(bvh_player)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(bvh STATIC bvh.cpp)
target_sources(bvh PUBLIC bvh.h)
target_link_libraries(bvh OpenGL::GL GLUT::GLUT)
target_include_directories(bvh PUBLIC ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
message(STATUS "OpenGL include dir: ${OPENGL_INCLUDE_DIR}")
message(STATUS "GLUT include dir: ${GLUT_INCLUDE_DIR}")

add_executable(bvh_player bvh_player.cpp)
target_link_libraries(bvh_player bvh)
