#include <stdio.h>

#include "mst.hpp"

__global__ void relax_kernel() {};

void start_kernel()
{
    relax_kernel<<<1, 1>>>();
}