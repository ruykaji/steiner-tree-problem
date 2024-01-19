cmake_minimum_required(VERSION 3.25.2)
project(steiner_tree_problem_gpu C CXX CUDA)

# Enable testing
enable_testing()

# Set C++ standard for the target
add_executable(${PROJECT_NAME} src/main.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE c_std_17 cxx_std_20 cuda_std_20)

# Set C++ standard for the library
add_library(${PROJECT_NAME}_lib src/main.cpp)
target_compile_features(${PROJECT_NAME}_lib PRIVATE c_std_17 cxx_std_20 cuda_std_20)

# Include directories
set(TARGET_DIRECTORIES
    ${CMAKE_SOURCE_DIR}/src
)

target_include_directories(${PROJECT_NAME} PRIVATE ${TARGET_DIRECTORIES})
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${TARGET_DIRECTORIES})

# Add subdirectory
add_subdirectory(${CMAKE_SOURCE_DIR}/src)
add_subdirectory(${CMAKE_SOURCE_DIR}/tests)