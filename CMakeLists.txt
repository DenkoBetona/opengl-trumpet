cmake_minimum_required(VERSION 3.16)
project(trumpet C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# GLFW (window + keyboard)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw GIT_REPOSITORY https://github.com/glfw/glfw.git GIT_TAG 3.4)
FetchContent_MakeAvailable(glfw)

# miniaudio (single header)
set(MINIAUDIO_DIR ${CMAKE_BINARY_DIR}/miniaudio)
if(NOT EXISTS ${MINIAUDIO_DIR}/miniaudio.h)
  file(DOWNLOAD https://raw.githubusercontent.com/mackron/miniaudio/0.11.21/miniaudio.h
       ${MINIAUDIO_DIR}/miniaudio.h SHOW_PROGRESS)
endif()

find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)

add_executable(trumpet main.cpp)
target_include_directories(trumpet PRIVATE ${MINIAUDIO_DIR})
target_link_libraries(trumpet PRIVATE glfw OpenGL::GL Threads::Threads ${CMAKE_DL_LIBS})
if(MSVC)
  target_compile_definitions(trumpet PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
