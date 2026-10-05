# Oldest CMake that can build this project; always the first line.
cmake_minimum_required(VERSION 3.15)
# Name the project and say it is written in C++ (CXX).
project(sts3215_arm CXX)

# Compile as C++17, and fail rather than silently fall back to an older standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # lets editors understand the code
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)       # optimized unless asked otherwise
endif()
# Compiler warnings for every target.
add_compile_options(-Wall -Wextra)

# Make a library called driver_core from these files
add_library(driver_core STATIC
    src/bus/sts3215.cpp src/kinematics/kinematics.cpp src/kinematics/calibration.cpp
    src/motion/recording.cpp)
# Anything that uses driver_core can #include headers from src/.
target_include_directories(driver_core PUBLIC src)

# Make a program called servo_tool from these source files...
add_executable(servo_tool
    src/main.cpp
    src/motion/arm.cpp src/motion/paths.cpp
    src/commands/record_replay.cpp src/commands/calibrate.cpp src/commands/touch.cpp
    src/commands/unlock.cpp
    src/bus/serial_port.cpp src/bus/connection.cpp)
# ...and link in the driver_core library (the protocol and recording code).
target_link_libraries(servo_tool PRIVATE driver_core)

# Optional MuJoCo simulation (`servo_tool --sim`), built when MuJoCo and GLFW are installed:
# brew install --cask mujoco && brew install glfw
set(MUJOCO_FRAMEWORKS /Applications/MuJoCo.app/Contents/Frameworks)
file(GLOB MUJOCO ${MUJOCO_FRAMEWORKS}/mujoco.framework/Versions/A/libmujoco.*.dylib)
find_package(glfw3 QUIET)
if(MUJOCO AND glfw3_FOUND)
    target_sources(servo_tool PRIVATE src/sim/sim.cpp)
    target_compile_options(servo_tool PRIVATE -F${MUJOCO_FRAMEWORKS})
    target_compile_definitions(servo_tool PRIVATE WITH_MUJOCO
        SIM_SCENE="${CMAKE_CURRENT_SOURCE_DIR}/sim/scene.xml")
    target_link_libraries(servo_tool PRIVATE ${MUJOCO} glfw)
    set_target_properties(servo_tool PROPERTIES BUILD_RPATH ${MUJOCO_FRAMEWORKS})
    message(STATUS "MuJoCo found: servo_tool --sim is available")
endif()

# Tests: a second program, built from the test files plus the same library.
enable_testing()
add_executable(driver_tests tests/test_bus.cpp tests/test_tools.cpp tests/test_kinematics.cpp)
target_include_directories(driver_tests PRIVATE third_party)   # where doctest.h lives
target_link_libraries(driver_tests PRIVATE driver_core)
# Register it with ctest, so `ctest --test-dir build` runs it.
add_test(NAME driver_tests COMMAND driver_tests)
