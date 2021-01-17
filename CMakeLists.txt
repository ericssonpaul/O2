cmake_minimum_required(VERSION 3.0)
project(O2)

set (CMAKE_CXX_STANDARD 11)

option(BUILD_TESTS    "Build tests"    ON)

file (GLOB O2src
    "O2.cpp"
)
add_library(O2 ${O2src})
target_include_directories(O2 PRIVATE .)

set (O2_LIBRARIES O2)
set (O2_DIRECTORIES .)

if (BUILD_TESTS)
    file (GLOB testSrc
        "tests/*.cpp"
    )
    add_executable(tests ${testSrc})
    target_include_directories(tests PRIVATE ${O2_DIRECTORIES})
    target_link_libraries(tests ${O2_LIBRARIES})
endif ()