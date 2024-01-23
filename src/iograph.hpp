#ifndef __IOGRAPH_HPP__
#define __IOGRAPH_HPP__

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "graph.hpp"

/**
 * @class ReadGraph
 * @brief Interface class for graph operations.
 *
 * This class provides an interface for graph input operations. It reads graph
 * data from a file, parses it, and constructs a Graph object.
 */
class ReadGraph {
public:
    ReadGraph() = default;
    ~ReadGraph() = default;

    /**
     * @brief Reads a graph from a file and constructs a Graph object.
     *
     * @param t_file_name The name of the file to read from.
     * @return Graph The constructed graph.
     * @throws std::runtime_error If the file cannot be opened or parsed.
     */
    InGraph operator()(const std::string& t_file_name)
    {
        std::ifstream in_file(t_file_name);

        if (!in_file) {
            throw std::runtime_error("Can't open file: " + t_file_name);
        }

        m_line_number = 0;
        m_graph.nodes.clear();
        m_graph.terminal_nodes.clear();

        std::string line;

        while (std::getline(in_file, line)) {
            ++m_line_number;

            if (line.empty()) {
                continue;
            }

            std::istringstream ss(line);
            parse_line(ss);
        }

        in_file.close();

        return m_graph;
    };

    /**
     * @brief Reads a graph from a inline graph description.
     *
     * @param t_inline_file The inlined graph.
     * @return Graph The constructed graph.
     */
    InGraph operator()(std::istringstream& t_inline_file)
    {
        m_line_number = 0;
        m_graph.nodes.clear();
        m_graph.terminal_nodes.clear();

        std::string line;

        while (std::getline(t_inline_file, line)) {
            ++m_line_number;

            if (line.empty()) {
                continue;
            }

            std::istringstream ss(line);
            parse_line(ss);
        }

        return m_graph;
    };

private:
    /**
     * @brief Parses a single line from the input file.
     *
     * @param t_ss Reference to a string stream containing the line to parse.
     * @param t_graph Reference to the graph being constructed.
     */
    void parse_line(std::istringstream& t_ss)
    {
        std::string token;
        t_ss >> token;

        if (token == "Nodes") {
            parse_number_nodes(t_ss);
        } else if (token == "Edges") {
            parse_number_edges(t_ss);
        } else if (token == "Terminals") {
            parse_number_terminals(t_ss);
        } else if (token == "E") {
            parse_edge(t_ss);
        } else if (token == "T") {
            parse_terminal(t_ss);
        }
    };

    /**
     * @brief Validates the stream state after attempting to read from it.
     *
     * @param t_ss The string stream to validate.
     * @throws std::runtime_error If the stream is in a failed state.
     */
    void validate_stream(const std::istringstream& t_ss) const
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
    void validate_bounds(int32_t t_node_index, int32_t t_size) const
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
     * @throws std::runtime_error If the nodes have already been initialized or if the input is invalid.
     */
    void parse_number_nodes(std::istringstream& t_ss)
    {
        if (!m_graph.nodes.empty()) {
            throw std::runtime_error("Attempt to set size after adding a node at line: " + std::to_string(m_line_number));
        }

        int32_t num_nodes;
        t_ss >> num_nodes;

        validate_stream(t_ss);

        m_graph.nodes.reserve(num_nodes);
    }

    /**
     * @brief Parses the number of edges from the input stream and initializes the graph's node container.
     *
     * This method should be called when a "Edges" token is encountered in the input file. It reserves space in the
     * graph's node map based on the number of edges specified.
     *
     * @param t_ss Reference to the input string stream.
     * @throws std::runtime_error If the edges have already been initialized or if the input is invalid.
     */
    void parse_number_edges(std::istringstream& t_ss)
    {
        if (!m_graph.nodes.empty()) {
            throw std::runtime_error("Attempt to set size after adding a edge at line: " + std::to_string(m_line_number));
        }

        int32_t num_edges;
        t_ss >> num_edges;

        validate_stream(t_ss);

        m_graph.map_edge_weight.reserve(num_edges);
    }

    /**
     * @brief Parses the number of terminal nodes from the input stream and initializes the graph's terminal node container.
     *
     * This method should be called when a "Terminals" token is encountered in the input file. It reserves space in the
     * graph's terminal node list based on the number of terminal nodes specified.
     *
     * @param t_ss Reference to the input string stream.
     * @throws std::runtime_error If the terminal nodes have already been initialized or if the input is invalid.
     */
    void parse_number_terminals(std::istringstream& t_ss)
    {
        if (!m_graph.terminal_nodes.empty()) {
            throw std::runtime_error("Attempt to set size after adding a node at line: " + std::to_string(m_line_number));
        }

        int32_t num_terminals;
        t_ss >> num_terminals;

        validate_stream(t_ss);

        m_graph.terminal_nodes.reserve(num_terminals);
    }

    /**
     * @brief Parses an edge from the input stream and adds it to the graph.
     *
     * This method should be called when an "E" token is encountered in the input file. It reads the edge data (from node,
     * to node, and weight) and adds the edge to the graph.
     *
     * @param t_ss Reference to the input string stream.
     * @throws std::runtime_error If the input is invalid or if the node indices are out of bounds.
     */
    void parse_edge(std::istringstream& t_ss)
    {
        int32_t from_node, to_node;
        double weight;

        t_ss >> from_node >> to_node >> weight;

        validate_stream(t_ss);
        validate_bounds(from_node, static_cast<int32_t>(m_graph.nodes.bucket_count()));
        validate_bounds(to_node, static_cast<int32_t>(m_graph.nodes.bucket_count()));

        m_graph.nodes[from_node].emplace_back(Edge(weight, from_node, to_node));
        m_graph.nodes[to_node].emplace_back(Edge(weight, to_node, from_node));
        m_graph.map_edge_weight[ordered_pair(from_node, to_node)] = weight;
    }

    /**
     * @brief Parses a terminal node from the input stream and adds it to the graph's list of terminal nodes.
     *
     * This method should be called when a "T" token is encountered in the input file. It reads the terminal node index
     * and adds it to the graph's list of terminal nodes.
     *
     * @param t_ss Reference to the input string stream.
     * @throws std::runtime_error If the input is invalid or if the node index is out of bounds.
     */
    void parse_terminal(std::istringstream& t_ss)
    {
        int32_t terminal_node;
        t_ss >> terminal_node;

        validate_stream(t_ss);
        validate_bounds(terminal_node, static_cast<int32_t>(m_graph.nodes.bucket_count()));

        m_graph.terminal_nodes.insert(terminal_node);
    }

private:
    int32_t m_line_number {}; ///> Line number tracker for input file parsing.
    InGraph m_graph {}; ///> In graph that will be constructed.
};

/**
 * @class WriteGraph
 * @brief A class to write graph information to a file.
 *
 * WriteGraph provides functionality to output the details of a graph,
 * including its edges, to a specified file. It also returns some statistics
 * about the graph, such as the total number of nodes, edges, and the total weight.
 */
class WriteGraph {
public:
    WriteGraph() = default;
    ~WriteGraph() = default;

    /**
     * @brief Overloaded function call operator to write graph data to a file.
     *
     * @param t_graph A constant reference to the OutGraph object representing the graph.
     * @param t_file_name A constant reference to a string representing the name of the output file.
     * @return std::unordered_map<std::string, std::string> A map containing statistics about the graph like total nodes, total edges, and total weight.
     * @throws std::runtime_error if the file cannot be opened.
     */
    std::unordered_map<std::string, std::string> operator()(const OutGraph& t_graph, const std::string& t_file_name)
    {
        std::ofstream out_file(t_file_name);

        if (!out_file) {
            throw std::runtime_error("Can't open file: " + t_file_name);
        }

        std::unordered_map<std::string, std::string> results {};
        std::unordered_set<int32_t> total_nodes {};
        int32_t total_edges {};

        for (const auto& edge : t_graph.first) {
            total_edges += 1;
            total_nodes.insert(edge.first);
            total_nodes.insert(edge.second);
            out_file << "E " << edge.first << ' ' << edge.second << '\n';
        }

        results["total_nodes"] = std::to_string(total_nodes.size());
        results["total_edges"] = std::to_string(total_edges);
        results["total_weight"] = std::to_string(t_graph.second);

        return results;
    };

    /**
     * @brief Overloaded function call operator to write graph data to a file.
     *
     * @param t_graph A constant reference to the OutGraph object representing the graph.
     * @param t_file_name A constant reference to a string representing the name of the output file.
     * @return std::unordered_map<std::string, std::string> A map containing statistics about the graph like total nodes, total edges, and total weight.
     * @throws std::runtime_error if the file cannot be opened.
     */
    std::string operator()(const OutGraph& t_graph)
    {
        std::ostringstream out_line {};

        for (const auto& edge : t_graph.first) {
            out_line << "E " << edge.first << ' ' << edge.second << '\n';
        }

        return out_line.str();
    };
};

#endif