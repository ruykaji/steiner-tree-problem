cmake_minimum_required(VERSION 3.25.2)
project(steiner_tree_problem LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
set(TARGET_DIRECTORIES 
    src)
    
add_executable(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${TARGET_DIRECTORIES})

find_package(GTest CONFIG QUIET)

if(NOT GTest_FOUND)
    message(WARNING "Google Test not found, testing will be skipped!")
else()
    # Set C++ standard for the library
    add_library(${PROJECT_NAME}_lib src/lib.cpp)
    target_include_directories(${PROJECT_NAME}_lib PUBLIC ${TARGET_DIRECTORIES})

    # Enable testing
    enable_testing()
    add_subdirectory(tests)
    include_directories(${GTEST_INCLUDE_DIRS})
endif()