cmake_minimum_required(VERSION 3.10)

# set compiler to g++
set(CMAKE_CXX_COMPILER g++)

# set the project name and version
project(OdomSim)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(include)
link_directories(lib)

# set the executable path
add_executable(OdomSim 
    src/Main.cpp
    src/autonomous.cpp
    src/control.cpp
    src/drive.cpp
    src/glad.C
    src/macros.cpp
    src/macros.cpp
    src/tracking.cpp
)

target_link_libraries (OdomSim LINK_PUBLIC glfw3 opengl32 gdi32)