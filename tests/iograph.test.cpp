#include <filesystem>
#include <gtest/gtest.h>

#include "iograph.hpp"

TEST(Input_Graph, Parse_File)
{
    IGraph input_graph {};

    if (std::filesystem::exists("../graph.txt")) {
        Graph graph = input_graph.input("../graph.txt");

        EXPECT_TRUE(!graph.nodes.empty());
        EXPECT_TRUE(!graph.terminal_nodes.empty());
    }

    SUCCEED();
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}   