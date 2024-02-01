cmake_minimum_required(VERSION 3.25.2)
project(steiner_tree_problem_gpu LANGUAGES CXX CUDA)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set CUDA standart
set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Include directories
set(TARGET_DIRECTORIES 
    src 
    src/cpu 
    src/cuda
    )

    
add_executable(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${TARGET_DIRECTORIES})
add_subdirectory(src/cuda)

# Try to enable testing if there is google test library.
# find_package(GTest CONFIG QUIET)

# if(NOT GTest_FOUND)
#     message(WARNING "Google Test not found, testing will be skipped!")
# else()
#     # Set C++ standard for the library
#     add_library(${PROJECT_NAME}_lib src/lib.cpp)
#     set_target_properties(${PROJECT_NAME}_lib PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
#     target_include_directories(${PROJECT_NAME}_lib PUBLIC ${TARGET_DIRECTORIES})

#     # Enable testing
#     enable_testing()
#     add_subdirectory(tests)
#     include_directories(${GTEST_INCLUDE_DIRS})
# endif()