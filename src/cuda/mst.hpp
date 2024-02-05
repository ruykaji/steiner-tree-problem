#ifndef __CUDA_MST_CUH__
#define __CUDA_MST_CUH__

#include <cstdint>
#include <thrust/device_vector.h>

#include "graph.hpp"

struct Request {
    int32_t node {};
    int32_t cost {};

    bool operator<(const Request& t_rhs) const { cost < t_rhs.cost; };
};

#ifdef __CUDACC__
template <typename T>
using Tent = thrust::device_vector<T>;

template <typename T>
using Bukets = thrust::device_vector<thrust::device_vector<T>>;

__global__ void relax_kernel(int32_t t_delta, int32_t t_w, int32_t t_x, Tent<int32_t>& t_tent, Bukets<int32_t>& t_bukets);
#endif

void start_kernel();

#endif