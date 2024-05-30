cmake_minimum_required(VERSION 3.10)
project(Verlet)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Paths
set(RAYLIB_DIR ${CMAKE_SOURCE_DIR}/deps/raylib-master/build/raylib)
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/deps/imgui-master)
set(RLIMGUI_DIR ${CMAKE_SOURCE_DIR}/deps/rlImGui)
set(CUSTOM_SRCS ${CMAKE_SOURCE_DIR}/src)

# Include directories
include_directories(${RAYLIB_DIR}/include)
include_directories(${IMGUI_DIR})
include_directories(${RLIMGUI_DIR})
include_directories(${CUSTOM_SRCS})

# Your source files
set(SOURCES
    ${CUSTOM_SRCS}/verlet_integration.cpp
    ${CUSTOM_SRCS}/physics.cpp
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${RLIMGUI_DIR}/rlImGui.cpp) 

# Create executable
add_executable(Verlet ${SOURCES})

# Platform-specific settings
if (APPLE)
    # Link libraries for macOS
    target_link_libraries(Verlet PRIVATE ${RAYLIB_DIR}/libraylib.a "-framework OpenGL" "-framework Cocoa" "-framework IOKit" "-framework CoreVideo")
elseif (UNIX)
    # Link libraries for Linux
    target_link_libraries(Verlet PRIVATE ${RAYLIB_DIR}/libraylib.a m GL glfw dl pthread)
elseif (WIN32)
    # Link libraries for Windows
    target_link_libraries(Verlet PRIVATE ${RAYLIB_DIR}/libraylib.a opengl32 gdi32 winmm shell32)
endif()

# Add raylib definitions
target_compile_definitions(Verlet PRIVATE -DUSE_RAYLIB)
