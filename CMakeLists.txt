cmake_minimum_required(VERSION 3.10)
project(pslg_extruder)

set(CMAKE_CXX_STANDARD 17)

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glm REQUIRED)
find_package(Boost REQUIRED)

add_subdirectory(${PROJECT_SOURCE_DIR}/lib/predicates EXCLUDE_FROM_ALL)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/nanovg EXCLUDE_FROM_ALL)

include_directories(
        ${OPENGL_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/lib/gl/include
        ${PROJECT_SOURCE_DIR}/lib/predicates/include
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
        vec.hpp
        ${CMAKE_CURRENT_BINARY_DIR}/shaders.cpp
        pslg.hpp
        shader.hpp
        ${PROJECT_SOURCE_DIR}/lib/gl/src/glad.c)
target_link_libraries(pslg_extruder
        predicates
        glfw
        nanovg
        ${OPENGL_gl_LIBRARY}
        ${Boost_LIBRARIES})
