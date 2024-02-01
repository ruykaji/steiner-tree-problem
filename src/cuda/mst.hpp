#ifndef __CUDA_MST_CUH__
#define __CUDA_MST_CUH__

#include <cstdint>
#include <vector>

#include "graph.hpp"

struct Request {
    int32_t node {};
    int32_t cost {};
};

#ifdef __CUDACC__
__global__ void relax_kernel();
#endif

void start_kernel();

#endif