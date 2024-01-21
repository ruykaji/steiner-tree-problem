#include <filesystem>
#include <gtest/gtest.h>

#include "cpu/mst.hpp"
#include "iograph.hpp"

/**
 * @brief Custom class for parameterized tests.
 */
class MSTOnCPUTrackTest : public testing::TestWithParam<std::string> {
};

/**
 * @brief Retrieves the file paths from the given directories.
 *
 * @param t_directories A vector of strings representing the directory paths to search.
 * @return std::vector<std::string> A vector containing the paths of all regular files found in the specified directories.
 */
std::vector<std::string> get_file_path(const std::vector<std::string>& t_directories)
{
    std::vector<std::string> file_paths;

    for (const auto& directory : t_directories) {
        if (std::filesystem::exists(directory)) {
            for (const auto& entry : std::filesystem::directory_iterator(directory)) {
                if (std::filesystem::is_regular_file(entry.status())) {
                    file_paths.push_back(entry.path().string());
                }
            }
        }
    }

    std::sort(file_paths.begin(), file_paths.end());
    return file_paths;
}

/**
 * @brief Converts an out-graph structure to an adjacency list representation.
 *
 * @param out_graph The graph structure to be converted.
 * @return std::unordered_map<int32_t, std::vector<int32_t>> An adjacency list representation of the `out-graph`.
 */
std::unordered_map<int32_t, std::vector<int32_t>> to_adjacency_list(const OutGraph& out_graph)
{
    std::unordered_map<int32_t, std::vector<int32_t>> adj_list {};

    for (const auto& edge : out_graph.first) {
        adj_list[edge.first].emplace_back(edge.second);
        adj_list[edge.second].emplace_back(edge.first);
    }

    return adj_list;
}

/**
 * @brief Performs a depth-first search (DFS) to check for cycles in a graph.
 *
 * @param node The node to start the DFS from.
 * @param visited A set of nodes that have been visited.
 * @param rec_stack A set of nodes that are currently in the recursion stack.
 * @param adj_list The adjacency list representation of the graph.
 * @return bool True if a cycle is detected, otherwise False.
 */
bool dfs(int32_t node, std::unordered_map<int32_t, bool>& visited, std::unordered_map<int32_t, bool>& rec_stack, const std::unordered_map<int32_t, std::vector<int32_t>>& adj_list)
{
    if (!visited[node]) {
        visited[node] = true;
        rec_stack[node] = true;

        for (int32_t neighbor : adj_list.at(node)) {
            if (!visited[node] && dfs(neighbor, visited, rec_stack, adj_list)) {
                return true;
            } else if (rec_stack[neighbor]) {
                return true;
            }
        }
    }

    rec_stack[node] = false;
    return false;
}

/**
 * @brief Checks if a graph represented by an adjacency list contains a cycle.
 *
 * @param adj_list The adjacency list representation of the graph.
 * @return bool True if the graph is cyclic, otherwise False.
 */
bool is_cyclic(const std::unordered_map<int32_t, std::vector<int32_t>>& adj_list)
{
    std::unordered_map<int32_t, bool> visited {};
    std::unordered_map<int32_t, bool> rec_stack {};

    for (const auto& [node, _] : adj_list) {
        if (dfs(node, visited, rec_stack, adj_list)) {
            return true;
        }
    }
    return false;
}

TEST_P(MSTOnCPUTrackTest, Tracks)
{
    std::string file_path = GetParam();
    ReadGraph reader {};
    CpuMST mst {};

    if (std::filesystem::is_regular_file(file_path)) {
        InGraph in_graph = reader(file_path);
        OutGraph out_graph = mst(in_graph);

        EXPECT_FALSE(is_cyclic(to_adjacency_list(out_graph)));
    } else {
        FAIL() << "File does not exist: " << file_path;
    }
}

INSTANTIATE_TEST_SUITE_P(
    Default, MSTOnCPUTrackTest,
    testing::ValuesIn(get_file_path({ "../../tests/files/Track1/", "../../tests/files/Track2/", "../../tests/files/Track3/" })));

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}