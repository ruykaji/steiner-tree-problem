# Add source files
set(CUDA_FILES mst.cu)

# Include source files
target_sources(${PROJECT_NAME} PRIVATE ${CUDA_FILES}) # Main project

if(GTest_Found)
    target_sources(${PROJECT_NAME}_lib PUBLIC ${CUDA_FILES}) # Library
endif()