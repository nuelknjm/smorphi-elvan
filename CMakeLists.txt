cmake_minimum_required(VERSION 3.16)
project(smorphi_navigation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(smorphi_navigation
    src/main.cpp
    src/Sensors.cpp
    src/Localization.cpp
    src/Planning.cpp
    src/StateMachine.cpp
)

target_include_directories(smorphi_navigation PRIVATE include)

# Threads is needed on some platforms for std::this_thread::sleep_for.
find_package(Threads REQUIRED)
target_link_libraries(smorphi_navigation PRIVATE Threads::Threads)
