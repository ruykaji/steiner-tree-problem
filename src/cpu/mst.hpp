#ifndef __CPU_MST_HPP__
#define __CPU_MST_HPP__

#include <queue>

#include "disjoin_set.hpp"
#include "graph.hpp"

class CPU_MST {
public:
    CPU_MST() = default;
    ~CPU_MST() = default;

    Graph const& make(const Graph& t_input_graph)
    {
        std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> priority_queue {};


    };

private:
    Graph m_graph {};
};

#endif