cmake_minimum_required(VERSION 3.26)
project(formal)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

include_directories(${CMAKE_SOURCE_DIR})




SET(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FETCHCONTENT_MAKEAVAILABLE(googletest)



enable_testing()

add_executable(testing
        tests/tests.cpp
)




target_link_libraries(
        testing
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(testing)
