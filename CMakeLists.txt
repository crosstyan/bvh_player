cmake_minimum_required(VERSION 3.20)
project(bvh_player)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(bvh STATIC bvh.cpp)
target_sources(bvh PUBLIC bvh.h)

if (APPLE)
    target_link_libraries(bvh OpenGL::GL GLUT::GLUT)
elseif (UNIX)
    target_link_libraries(bvh OpenGL::GL GLUT::GLUT GLU)
else ()
    target_link_libraries(bvh OpenGL::GL GLUT::GLUT)
endif ()

target_include_directories(bvh PUBLIC ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
message(STATUS "OpenGL include dir: ${OPENGL_INCLUDE_DIR}")
message(STATUS "GLUT include dir: ${GLUT_INCLUDE_DIR}")

add_executable(bvh_player bvh_player.cpp)
target_link_libraries(bvh_player bvh)

add_executable(print print.cpp)
target_link_libraries(print bvh)
