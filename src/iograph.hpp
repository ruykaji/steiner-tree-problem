#ifndef __IOGRAPH_HPP__
#define __IOGRAPH_HPP__

#include <memory>
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
     * @brief Reads a graph from a input stream(file or stdin).
     *
     * @param t_in_stream The input graph stream.
     * @return Graph The constructed graph.
     */
    std::unique_ptr<InGraph> operator()(std::istream& t_in_stream)
    {
        m_line_number = 0;
        m_graph = std::make_unique<InGraph>();

        std::string line;

        while (std::getline(t_in_stream, line)) {
            ++m_line_number;

            if (line.empty()) {
                continue;
            }

            m_ss.clear();
            m_ss.str(line);

            parse_line();
        }

        return std::move(m_graph);
    };

private:
    /**
     * @brief Parses a single line from the input file.
     *
     * @param t_graph Reference to the graph being constructed.
     */
    void parse_line()
    {
        std::string token;
        m_ss >> token;

        if (token == "Nodes") {
            parse_number_nodes();
        } else if (token == "Edges") {
            parse_number_edges();
        } else if (token == "Terminals") {
            parse_number_terminals();
        } else if (token == "E") {
            parse_edge();
        } else if (token == "T") {
            parse_terminal();
        }
    };

    /**
     * @brief Validates the stream state after attempting to read from it.
     *
     * @throws std::runtime_error If the stream is in a failed state.
     */
    void validate_stream() const
    {
        if (m_ss.fail()) {
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
    void validate_bounds(std::size_t t_node_index, std::size_t t_size) const
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
     * @throws std::runtime_error If the nodes have already been initialized or if the input is invalid.
     */
    void parse_number_nodes()
    {
        if (!m_graph->nodes.empty()) {
            throw std::runtime_error("Attempt to set size after adding a node at line: " + std::to_string(m_line_number));
        }

        int32_t num_nodes;
        m_ss >> num_nodes;

        validate_stream();

        m_graph->nodes.reserve(num_nodes);
    }

    /**
     * @brief Parses the number of edges from the input stream and initializes the graph's node container.
     *
     * This method should be called when a "Edges" token is encountered in the input file. It reserves space in the
     * graph's node map based on the number of edges specified.
     *
     * @throws std::runtime_error If the edges have already been initialized or if the input is invalid.
     */
    void parse_number_edges()
    {
        if (!m_graph->nodes.empty()) {
            throw std::runtime_error("Attempt to set size after adding a edge at line: " + std::to_string(m_line_number));
        }

        int32_t num_edges;
        m_ss >> num_edges;

        validate_stream();

        m_graph->map_edge_weight.reserve(num_edges);
    }

    /**
     * @brief Parses the number of terminal nodes from the input stream and initializes the graph's terminal node container.
     *
     * This method should be called when a "Terminals" token is encountered in the input file. It reserves space in the
     * graph's terminal node list based on the number of terminal nodes specified.
     *
     * @throws std::runtime_error If the terminal nodes have already been initialized or if the input is invalid.
     */
    void parse_number_terminals()
    {
        if (!m_graph->terminal_nodes.empty()) {
            throw std::runtime_error("Attempt to set size after adding a node at line: " + std::to_string(m_line_number));
        }

        int32_t num_terminals;
        m_ss >> num_terminals;

        validate_stream();

        m_graph->terminal_nodes.reserve(num_terminals);
    }

    /**
     * @brief Parses an edge from the input stream and adds it to the graph.
     *
     * This method should be called when an "E" token is encountered in the input file. It reads the edge data (from node,
     * to node, and weight) and adds the edge to the graph.
     *
     * @throws std::runtime_error If the input is invalid or if the node indices are out of bounds.
     */
    void parse_edge()
    {
        int32_t from_node;
        int32_t to_node;
        double weight;

        m_ss >> from_node >> to_node >> weight;

        validate_stream();
        validate_bounds(from_node, m_graph->nodes.bucket_count());
        validate_bounds(to_node, m_graph->nodes.bucket_count());

        m_graph->nodes[from_node].emplace_back(Edge(weight, from_node, to_node));
        m_graph->nodes[to_node].emplace_back(Edge(weight, to_node, from_node));
        m_graph->map_edge_weight[ordered_pair(from_node, to_node)] = weight;
    }

    /**
     * @brief Parses a terminal node from the input stream and adds it to the graph's list of terminal nodes.
     *
     * This method should be called when a "T" token is encountered in the input file. It reads the terminal node index
     * and adds it to the graph's list of terminal nodes.
     *
     * @throws std::runtime_error If the input is invalid or if the node index is out of bounds.
     */
    void parse_terminal()
    {
        int32_t terminal_node;
        m_ss >> terminal_node;

        validate_stream();
        validate_bounds(terminal_node, m_graph->nodes.bucket_count());

        m_graph->terminal_nodes.insert(terminal_node);
    }

private:
    std::size_t m_line_number {}; ///> Line number tracker for input file parsing.
    std::istringstream m_ss {}; ///> String stream for line processing.
    std::unique_ptr<InGraph> m_graph {}; ///> In graph that will be constructed.
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
     * @brief Overloaded function call operator to write graph data to a output stream(file/stdout).
     *
     * @param t_graph A constant reference to the OutGraph object representing the graph.
     * @param t_out_stream A reference to a output stream
     */
    void operator()(std::unique_ptr<OutGraph> t_graph, std::ostream& t_out_stream)
    {
        for (const auto& edge : t_graph->first) {
            t_out_stream << "E " << edge.first << ' ' << edge.second << '\n';
        }
    };
};

#endif