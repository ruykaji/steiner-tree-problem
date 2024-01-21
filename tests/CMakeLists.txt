# Find google test package
# ==========================================================================
find_package(GTest CONFIG REQUIRED)
if(NOT GTest_FOUND)
    message(FATAL_ERROR "Google Test not found!")
endif()

# Include google test directories
# ==========================================================================
include_directories(${GTEST_INCLUDE_DIRS})

# Add tests
# ==========================================================================

set(TESTS
    iograph
    cpu_disjoin_set
    cpu_mst
)

# Iterate over each test
foreach(TEST_NAME IN LISTS TESTS)
set(TEST_SRC "${TEST_NAME}.test.cpp")

    # Add test source 
    add_executable(${TEST_NAME} ${TEST_SRC})
    target_compile_features(${TEST_NAME} PRIVATE cxx_std_20)

    # Include google test directory
    target_include_directories(${TEST_NAME} PRIVATE ${GTEST_INCLUDE_DIRS})

    # Link google test, pthread and main project libraries
    target_link_libraries(${TEST_NAME} PRIVATE GTest::gtest pthread ${PROJECT_NAME}_lib)

    # Add test
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME} --gtest_break_on_failure)
endforeach()