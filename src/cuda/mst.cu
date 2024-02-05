#include <chrono>
#include <cuda.h>
#include <cuda_runtime.h>
#include <iostream>
#include <stdio.h>
#include <vector>

#include "mst.hpp"

#define CHECK_CUDA(call)                                                        \
    {                                                                           \
        const cudaError_t error = call;                                         \
        if (error != cudaSuccess) {                                             \
            printf("Error: %s:%d, ", __FILE__, __LINE__);                       \
            printf("code: %d, reason: %s\n", error, cudaGetErrorString(error)); \
            exit(1);                                                            \
        }                                                                       \
    }

#define CHECK_KERNEL()                                                          \
    {                                                                           \
        const cudaError_t error = cudaGetLastError();                           \
        if (error != cudaSuccess) {                                             \
            printf("Kernel launch failure: %s:%d, ", __FILE__, __LINE__);       \
            printf("code: %d, reason: %s\n", error, cudaGetErrorString(error)); \
            exit(1);                                                            \
        }                                                                       \
    }

struct Edge {
    int32_t destination;
    int32_t weight;
};

void add_edge(std::vector<std::vector<Edge>>& t_graph, int32_t t_a, int32_t t_b, int32_t t_w)
{
    t_graph[t_a - 1].emplace_back(Edge { t_b - 1, t_w });
    t_graph[t_b - 1].emplace_back(Edge { t_a - 1, t_w });
}

// =============================== HELPERS ================================

namespace cuda_array {
template <typename T>
__global__ void set_at(T* t_array, T t_value, size_t t_index) { t_array[t_index] = t_value; }

template <typename T>
__global__ void set(T* t_array, T t_value, size_t t_size)
{
    uint32_t index = blockIdx.x * blockDim.x + threadIdx.x;
    uint32_t stride = blockDim.x * gridDim.x;

    for (size_t i = index; i < t_size; i += stride) {
        t_array[i] = t_value;
    }
}
};

// ========================================================================

__global__ void update_distance(int32_t t_source, int32_t* t_dist, Edge** t_graph)
{
    uint32_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    t_dist[t_graph[t_source][idx].destination] = t_graph[t_source][idx].weight;
}

__global__ void find_min_distance(size_t t_size, int32_t* t_u, int32_t* t_dist, int32_t* t_processed)
{
    int32_t min_distance = __INT_MAX__;
    *t_u = -1;

    for (int32_t i = 0; i < t_size; ++i) {
        if (t_processed[i] == -1 && t_dist[i] < min_distance) {
            min_distance = t_dist[i];
            *t_u = i;
        }
    }

    if (*t_u != -1) {
        t_processed[*t_u] = 1;
    }
}

__global__ void processed_edges(int32_t t_u, Edge** t_graph, int32_t t_edge_size, int32_t* t_dist, int32_t* t_processed, int32_t t_size)
{
    int32_t index = blockIdx.x * blockDim.x + threadIdx.x;
    int32_t stride = blockDim.x * gridDim.x;

    for (int32_t i = index; i < t_edge_size; i += stride) {
        if (t_processed[t_graph[t_u][i].destination] == -1) {
            atomicMin(&t_dist[t_graph[t_u][i].destination], t_dist[t_u] + t_graph[t_u][i].weight);
        };
    }
}

void calculate_distance(int32_t t_size, int32_t t_source, Edge** t_graph, int32_t* t_edges_count, int32_t* t_radius)
{

    int32_t* dist;
    CHECK_CUDA(cudaMalloc(&dist, t_size * sizeof(int32_t)));

    int32_t* processed;
    CHECK_CUDA(cudaMalloc(&processed, t_size * sizeof(int32_t)));
    cuda_array::set<<<((t_size + 256 - 1) / 256), 256>>>(processed, -1, t_size);
    cuda_array::set_at<<<1, 1>>>(processed, 1, t_source);

    int32_t* host_u = (int32_t*)malloc(sizeof(int32_t));
    int32_t* device_u;
    CHECK_CUDA(cudaMalloc(&device_u, sizeof(int32_t)));

    auto start = std::chrono::high_resolution_clock::now();

    cuda_array::set<<<((t_size + 256 - 1) / 256), 256>>>(dist, INT_MAX, t_size);
    cuda_array::set_at<<<1, 1>>>(dist, 0, t_source);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl;

    int32_t processed_counter = 1;

    update_distance<<<1, t_edges_count[t_source]>>>(t_source, dist, t_graph);

    while (processed_counter < t_size) {
        find_min_distance<<<1, 1>>>(t_size, device_u, dist, processed);
        CHECK_CUDA(cudaMemcpy(host_u, device_u, sizeof(int32_t), cudaMemcpyDeviceToHost));

        if (*host_u == -1) {
            break;
        }

        ++processed_counter;

        processed_edges<<<((t_edges_count[*host_u] + 256 - 1) / 256), 256>>>(*host_u, t_graph, t_edges_count[*host_u], dist, processed, t_size);
    }


    int32_t* host_dist = (int32_t*)malloc(t_size * sizeof(int32_t));
    cudaMemcpy(host_dist, dist, t_size * sizeof(int32_t), cudaMemcpyDeviceToHost);

    for (int32_t i = 0; i < t_size; ++i) {
        printf("Value %d: %d\n", i + 1, host_dist[i]);
    }

    // Free cuda
    cudaFree(dist);
    cudaFree(processed);
    cudaFree(device_u);

    // Free host
    free(host_u);
}