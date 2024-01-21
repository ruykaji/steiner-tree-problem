#include <gtest/gtest.h>

#include "cpu/mst.hpp"

TEST(MST_On_CPU, Run)
{
    Graph graph {};

    graph.nodes.reserve(6);
    graph.terminal_nodes.reserve(3);

    graph.nodes[1].push_back(Edge(10, 1, 3));
    graph.nodes[3].push_back(Edge(10, 3, 1));

    graph.nodes[1].push_back(Edge(1, 1, 5));
    graph.nodes[5].push_back(Edge(1, 5, 1));

    graph.nodes[1].push_back(Edge(1, 1, 6));
    graph.nodes[6].push_back(Edge(1, 6, 1));

    graph.nodes[2].push_back(Edge(10, 2, 5));
    graph.nodes[5].push_back(Edge(10, 5, 2));

    graph.nodes[2].push_back(Edge(1, 2, 6));
    graph.nodes[6].push_back(Edge(1, 6, 2));

    graph.nodes[3].push_back(Edge(1, 3, 4));
    graph.nodes[4].push_back(Edge(1, 4, 3));

    graph.nodes[3].push_back(Edge(1, 3, 5));
    graph.nodes[5].push_back(Edge(1, 5, 3));

    graph.nodes[5].push_back(Edge(10, 5, 6));
    graph.nodes[6].push_back(Edge(10, 6, 5));

    graph.terminal_nodes.insert(1);
    graph.terminal_nodes.insert(2);
    graph.terminal_nodes.insert(4);

    CpuMST mst;
    auto path_and_weight = mst.make(graph);

    EXPECT_EQ(path_and_weight.second, 5);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}