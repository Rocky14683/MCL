cmake_minimum_required(VERSION 3.28)
project(MCL)

set(CMAKE_C_COMPILER "/opt/homebrew/opt/llvm/bin/clang")
set(CMAKE_CXX_COMPILER "/opt/homebrew/opt/llvm/bin/clang++")


set(CMAKE_CXX_STANDARD 23)

include_directories(include)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include(FetchContent)
FetchContent_Declare(
        raylib
        GIT_REPOSITORY "https://github.com/raysan5/raylib.git"
        GIT_TAG "master"
        GIT_PROGRESS TRUE
)

FetchContent_Declare(
        rerun_sdk
        URL https://github.com/rerun-io/rerun/releases/download/0.23.2/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)

FetchContent_MakeAvailable(raylib)

find_package(Curses REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

add_executable(MCL src/main.cpp)

target_include_directories(MCL PRIVATE include ${GLFW_INCLUDE_DIRS} ${CURSES_INCLUDE_DIR})
target_link_libraries(MCL PUBLIC raylib rerun_sdk Eigen3::Eigen ${GLFW_LIBRARIES} ${CURSES_LIBRARIES})

FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)


enable_testing()

file(GLOB TEST_SOURCES test/*.cpp)
add_executable(GTEST ${TEST_SOURCES}
        test/test.cpp)

target_include_directories(GTEST PRIVATE include)

target_link_libraries(GTEST PUBLIC gtest gtest_main Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(GTEST DISCOVERY_TIMEOUT 60)
