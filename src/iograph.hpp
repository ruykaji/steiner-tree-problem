#ifndef __IOGRAPH_HPP__
#define __IOGRAPH_HPP__

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "edge.hpp"

/**
 * @brief Structure representing a graph.
 *
 * The Graph consists of nodes which are connected by edges. Each node is
 * represented by an integer identifier and stores a list of edges. Additionally,
 * some nodes are designated as terminal nodes.
 */
struct Graph {
    // Map of node IDs to their corresponding list of edges.
    std::unordered_map<int32_t, std::vector<Edge>> nodes {};

    // List of terminal node IDs.
    std::vector<int32_t> terminal_nodes {};
};

/**
 * @brief Interface class for graph operations.
 *
 * This class provides an interface for graph input operations. It reads graph
 * data from a file, parses it, and constructs a Graph object.
 */
class IGraph {
public:
    IGraph() = default;
    ~IGraph() = default;

    /**
     * @brief Reads a graph from a file and constructs a Graph object.
     *
     * @param t_file_name The name of the file to read from.
     * @return Graph The constructed graph.
     * @throws std::runtime_error If the file cannot be opened or parsed.
     */
    inline Graph input(const std::string& t_file_name)
    {
        std::ifstream in_file(t_file_name);

        if (!in_file) {
            throw std::runtime_error("Can't open file: " + t_file_name);
        }

        Graph graph;
        std::string line;

        m_line_number = 0;

        while (std::getline(in_file, line)) {
            ++m_line_number;

            if (line.empty()) {
                continue;
            }

            std::istringstream ss(line);
            parse_line(ss, graph);
        }

        return graph;
    };

private:
    /**
     * @brief Parses a single line from the input file.
     *
     * @param t_ss Reference to a string stream containing the line to parse.
     * @param t_graph Reference to the graph being constructed.
     */
    inline void parse_line(std::istringstream& t_ss, Graph& t_graph)
    {
        std::string token;
        t_ss >> token;

        if (token == "Nodes") {
            parse_number_nodes(t_ss, t_graph);
        } else if (token == "Terminals") {
            parse_number_terminals(t_ss, t_graph);
        } else if (token == "E") {
            parse_edge(t_ss, t_graph);
        } else if (token == "T") {
            parse_terminal(t_ss, t_graph);
        }
    };

    /**
     * @brief Validates the stream state after attempting to read from it.
     *
     * @param t_ss The string stream to validate.
     * @throws std::runtime_error If the stream is in a failed state.
     */
    inline void validate_stream(const std::istringstream& t_ss) const
    {
        if (t_ss.fail()) {
            throw std::runtime_error("Invalid input at line " + std::to_string(m_line_number));
        }
    }

    /**
     * @brief Validates if a node index is within the bounds of the graph.
     *
     * @param t_node_index The node index to validate.
     * @param t_size The size of the node container.
     * @throws std::runtime_error If the node index is out of bounds.
     */
    inline void validate_bounds(int32_t t_node_index, int32_t t_size) const
    {
        if (t_node_index > t_size) {
            throw std::runtime_error("Node index out of bounds at line " + std::to_string(m_line_number));
        }
    }

    /**
     * @brief Parses the number of nodes from the input stream and initializes the graph's node container.
     *
     * This method should be called when a "Nodes" token is encountered in the input file. It reserves space in the
     * graph's node map based on the number of nodes specified.
     *
     * @param t_ss Reference to the input string stream.
     * @param t_graph Reference to the graph being constructed.
     * @throws std::runtime_error If the nodes have already been initialized or if the input is invalid.
     */
    inline void parse_number_nodes(std::istringstream& t_ss, Graph& t_graph)
    {
        if (!t_graph.nodes.empty()) {
            throw std::runtime_error("Attempt to set size after adding a node at line: " + std::to_string(m_line_number));
        }

        int32_t num_nodes;
        t_ss >> num_nodes;

        validate_stream(t_ss);

        t_graph.nodes.reserve(num_nodes);
    }

    /**
     * @brief Parses the number of terminal nodes from the input stream and initializes the graph's terminal node container.
     *
     * This method should be called when a "Terminals" token is encountered in the input file. It reserves space in the
     * graph's terminal node list based on the number of terminal nodes specified.
     *
     * @param t_ss Reference to the input string stream.
     * @param t_graph Reference to the graph being constructed.
     * @throws std::runtime_error If the terminal nodes have already been initialized or if the input is invalid.
     */
    inline void parse_number_terminals(std::istringstream& t_ss, Graph& t_graph)
    {
        if (!t_graph.terminal_nodes.empty()) {
            throw std::runtime_error("Attempt to set size after adding a node at line: " + std::to_string(m_line_number));
        }

        int32_t num_terminals;
        t_ss >> num_terminals;

        validate_stream(t_ss);

        t_graph.terminal_nodes.reserve(num_terminals);
    }

    /**
     * @brief Parses an edge from the input stream and adds it to the graph.
     *
     * This method should be called when an "E" token is encountered in the input file. It reads the edge data (from node,
     * to node, and weight) and adds the edge to the graph.
     *
     * @param t_ss Reference to the input string stream.
     * @param t_graph Reference to the graph being constructed.
     * @throws std::runtime_error If the input is invalid or if the node indices are out of bounds.
     */
    inline void parse_edge(std::istringstream& t_ss, Graph& t_graph)
    {
        int32_t from_node, to_node;
        double weight;

        t_ss >> from_node >> to_node >> weight;
        
        validate_stream(t_ss);
        validate_bounds(from_node, static_cast<int32_t>(t_graph.nodes.bucket_count()));
        validate_bounds(to_node, static_cast<int32_t>(t_graph.nodes.bucket_count()));

        t_graph.nodes[from_node].push_back(Edge(weight, from_node, to_node));
    }

    /**
     * @brief Parses a terminal node from the input stream and adds it to the graph's list of terminal nodes.
     *
     * This method should be called when a "T" token is encountered in the input file. It reads the terminal node index
     * and adds it to the graph's list of terminal nodes.
     *
     * @param t_ss Reference to the input string stream.
     * @param t_graph Reference to the graph being constructed.
     * @throws std::runtime_error If the input is invalid or if the node index is out of bounds.
     */
    inline void parse_terminal(std::istringstream& t_ss, Graph& t_graph)
    {
        int32_t terminal_node;
        t_ss >> terminal_node;

        validate_stream(t_ss);
        validate_bounds(terminal_node, static_cast<int32_t>(t_graph.nodes.bucket_count()));

        t_graph.terminal_nodes.push_back(terminal_node);
    }

private:
    // Line number tracker for input file parsing.
    int32_t m_line_number {};
};

class OGraph {
public:
    OGraph() = default;
    ~OGraph() = default;

    inline void out(const Graph& t_graph, const std::string& t_file_name) {

    };

private:
    std::string m_file_name {};
};

#endif