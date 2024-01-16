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
set(TARGET_DIRECTORIES)

target_include_directories(${PROJECT_NAME} PRIVATE ${TARGET_DIRECTORIES})
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${TARGET_DIRECTORIES})

# Add subdirectory
add_subdirectory(${CMAKE_SOURCE_DIR}/src)
add_subdirectory(${CMAKE_SOURCE_DIR}/tests)

# Fetch JSON Library
include(FetchContent)
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.10.5
)
FetchContent_MakeAvailable(json)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC nlohmann_json::nlohmann_json)