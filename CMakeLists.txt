cmake_minimum_required(VERSION 3.2)
project(pslg_extruder)

set(CMAKE_CXX_STANDARD 17)

find_package(glfw3 3.3 REQUIRED)

add_subdirectory(${PROJECT_SOURCE_DIR}/lib/nanovg EXCLUDE_FROM_ALL)

include_directories(
        ${OPENGL_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/lib/gl/include
        ${PROJECT_SOURCE_DIR}/lib/nanovg)

set(shader_files
        ${PROJECT_SOURCE_DIR}/vertex.glsl
        ${PROJECT_SOURCE_DIR}/fragment.glsl)

add_executable(tostring tostring.cpp)

add_custom_command(
        OUTPUT  shaders.cpp
        COMMAND tostring ${CMAKE_CURRENT_BINARY_DIR}/shaders.cpp ${shader_files}
        DEPENDS tostring ${shader_files}
)

add_executable(pslg_extruder
        main.cpp
        pslg.hpp
        vec.hpp
        fixedvector.hpp
        ${CMAKE_CURRENT_BINARY_DIR}/shaders.cpp
        shader.hpp
        ${PROJECT_SOURCE_DIR}/lib/gl/src/glad.c)
target_link_libraries(pslg_extruder
        glfw
        nanovg)
