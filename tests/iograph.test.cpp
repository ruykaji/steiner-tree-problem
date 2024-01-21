#include <filesystem>
#include <gtest/gtest.h>

#include "iograph.hpp"

TEST(Input_Graph, Parse_File)
{
    ReadGraph reader {};

    if (std::filesystem::exists("../graph.txt")) {
        InGraph in_graph = reader("../graph.txt");

        EXPECT_TRUE(!in_graph.nodes.empty());
        EXPECT_TRUE(!in_graph.terminal_nodes.empty());
    }

    SUCCEED();
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}