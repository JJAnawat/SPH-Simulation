cmake_minimum_required(VERSION 3.20)
project(SPHSimulation)

set(CMAKE_CXX_STANDARD 17)

# GLFW - disable extras we don't need
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(external/glfw)

# GLAD
add_library(glad STATIC external/glad/src/glad.c)
target_include_directories(glad PUBLIC external/glad/include)

# ImGui (core + GLFW + OpenGL3 backends only)
set(IMGUI_DIR external/imgui)
add_library(imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
target_link_libraries(imgui PUBLIC glfw)

# Main executable
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_executable(SPHSimulation ${SRC_FILES})

target_include_directories(SPHSimulation PRIVATE
    src/
    external/glad/include
    external/glm
)

target_link_libraries(SPHSimulation PRIVATE glfw glad imgui)