#ifndef __IOGRAPH_HPP__
#define __IOGRAPH_HPP__

#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "graph.hpp"

constexpr std::size_t MAX_CHUNK_SIZE = 65536;

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

        std::size_t chunk_size = MAX_CHUNK_SIZE;
        std::size_t partial_length = 0;
        std::string chunk(chunk_size, ' ');

        while (t_in_stream) {
            t_in_stream.read(chunk.data() + partial_length, chunk_size - partial_length);

            if (!t_in_stream && !t_in_stream.eof()) {
                throw std::runtime_error("File has been corrupted!");
            }

            std::size_t bytes_read = t_in_stream.gcount() + partial_length;
            std::size_t last_new_line = chunk.rfind('\n', bytes_read - 1);

            if (last_new_line != std::string::npos && bytes_read != 0) {
                std::size_t left = 0;
                std::string_view chunk_view(chunk);

                while (left < last_new_line) {
                    std::size_t right = chunk.find('\n', left);

                    if (right == std::string::npos)
                        break;

                    ++m_line_number;
                    parse_line(chunk_view.substr(left, right - left));

                    left = right + 1;
                }

                partial_length = bytes_read - (last_new_line + 1);
                std::memmove(&chunk[0], &chunk[last_new_line + 1], partial_length);
            } else {
                partial_length = bytes_read;
            }
        }

        if (partial_length != 0) {
            parse_line(chunk.substr(0, partial_length));
        }

        return std::move(m_graph);
    };

private:
    /**
     * @brief Parses a single line from the input file.
     *
     * @param t_graph Reference to the graph being constructed.
     */
    void parse_line(std::string_view line)
    {
        std::string_view token = line.substr(0, line.find(' '));

        if (token == "Nodes") {
            parse_number_nodes(line);
        } else if (token == "Edges") {
            parse_number_edges(line);
        } else if (token == "Terminals") {
            parse_number_terminals(line);
        } else if (token == "E") {
            parse_edge(line);
        } else if (token == "T") {
            parse_terminal(line);
        }
    };

    /**
     * @brief Validates if a node index is within the bounds of the graph.
     *
     * @param t_node_index The node index to validate.
     * @param t_size The size of the node container.
     * @throws std::runtime_error If the node index is out of bounds.
     */
    void validate_bounds(std::size_t t_node_index, std::size_t t_size) const
    {
        if (t_node_index > t_size || t_node_index < 1) {
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
    void parse_number_nodes(std::string_view line)
    {
        if (!m_graph->nodes.empty()) {
            throw std::runtime_error("Attempt to set nodes size after adding a node at line: " + std::to_string(m_line_number));
        }

        int32_t num_nodes = std::atoi(line.data() + 6); // Assume that line starts with "Nodes "
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
    void parse_number_edges(std::string_view line)
    {
        if (!m_graph->nodes.empty()) {
            throw std::runtime_error("Attempt to set edges size after adding a edge at line: " + std::to_string(m_line_number));
        }

        int32_t num_edges = std::atoi(line.data() + 6); // Assume that line starts with "Edges "
        m_graph->edges.reserve(num_edges);
        // m_graph->map_edge_weight.reserve(num_edges);
    }

    /**
     * @brief Parses the number of terminal nodes from the input stream and initializes the graph's terminal node container.
     *
     * This method should be called when a "Terminals" token is encountered in the input file. It reserves space in the
     * graph's terminal node list based on the number of terminal nodes specified.
     *
     * @throws std::runtime_error If the terminal nodes have already been initialized or if the input is invalid.
     */
    void parse_number_terminals(std::string_view line)
    {
        if (!m_graph->terminal_nodes.empty()) {
            throw std::runtime_error("Attempt to set terminal size after adding a node at line: " + std::to_string(m_line_number));
        }

        int32_t num_terminals = std::atoi(line.data() + 9); // Assume that line starts with "Terminals "
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
    void parse_edge(std::string_view line)
    {
        int32_t from_node;
        int32_t to_node;
        double weight;

        {
            std::size_t left = 2; // Assume that line starts with "E "
            std::size_t right = 0;

            right = line.find(' ', left);
            from_node = std::atoi(line.substr(left, right - left).data());

            left = right + 1;
            right = line.find(' ', left);
            to_node = std::atoi(line.substr(left, right - left).data());

            left = right + 1;
            weight = std::atof(line.data() + left);
        }

        validate_bounds(from_node, m_graph->nodes.bucket_count());
        validate_bounds(to_node, m_graph->nodes.bucket_count());

        Edge new_edge(weight, std::min(from_node, to_node), std::max(from_node, to_node));

        m_graph->edges.emplace_back(new_edge);
        m_graph->nodes[from_node].emplace_back(&m_graph->edges.back());
        m_graph->nodes[to_node].emplace_back(&m_graph->edges.back());
        // m_graph->map_edge_weight[ordered_pair(from_node, to_node)] = weight;
    }

    /**
     * @brief Parses a terminal node from the input stream and adds it to the graph's list of terminal nodes.
     *
     * This method should be called when a "T" token is encountered in the input file. It reads the terminal node index
     * and adds it to the graph's list of terminal nodes.
     *
     * @throws std::runtime_error If the input is invalid or if the node index is out of bounds.
     */
    void parse_terminal(std::string_view line)
    {
        int32_t terminal_node = std::atoi(line.data() + 2); // Assume that line starts with "T "
        validate_bounds(terminal_node, m_graph->nodes.bucket_count());
        m_graph->terminal_nodes.insert(terminal_node);
    }

private:
    std::size_t m_line_number {}; ///> Line number tracker for input file parsing.
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
        for (auto it = t_graph->begin(); it != t_graph->end(); ++it) {
            t_out_stream << "E " << it->first << ' ' << it->second << std::endl;
        }

        t_out_stream << "EOF" << std::endl;
    };
};

#endif