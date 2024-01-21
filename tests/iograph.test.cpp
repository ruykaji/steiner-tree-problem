#include <filesystem>
#include <gtest/gtest.h>

#include "iograph.hpp"

TEST(Input_Graph, Parse_File)
{
    ReadGraph reader {};

    if (std::filesystem::exists("../../tests/files/Track1/instance001.gr")) {
        InGraph in_graph = reader("../../tests/files/Track1/instance001.gr");

        EXPECT_TRUE(!in_graph.nodes.empty());
        EXPECT_TRUE(!in_graph.terminal_nodes.empty());
    } else {
        FAIL() << "File does not exist: ../../tests/files/Track1/instance001.gr\n";
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}