#ifndef __GRAPH_HPP__
#define __GRAPH_HPP__

#include <cstdint>
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/**
 * @class Edge
 * @brief Class representing an edge in a graph.
 *
 * This class represents an edge with a weight, source, and destination.
 * It provides methods to get the weight, source, and destination of the edge,
 * as well as an overload of the greater-than operator for comparing edges.
 */
class Edge {
public:
    /**
     * @brief Construct a new Edge object.
     *
     * @param t_weight Weight of the edge.
     * @param t_source Source vertex of the edge.
     * @param t_destination Destination vertex of the edge.
     */
    explicit Edge(double t_weight, int32_t t_source, int32_t t_destination, int32_t t_prev1 = -1, int32_t t_prev2 = -1)
        : m_weight(t_weight)
        , m_source(t_source)
        , m_destination(t_destination)
        , m_prev_source(t_prev1)
        , m_prev_destination(t_prev2) {};

    ~Edge() = default;

    /**
     * @brief Get the weight of the edge.
     *
     * @return Value of the weight of the edge.
     */
    double get_weight() const noexcept { return m_weight; };

    /**
     * @brief Get the source vertex of the edge.
     *
     * @return Value of the source vertex of the edge.
     */
    int32_t get_source() const noexcept { return m_source; };

    /**
     * @brief Get the destination vertex of the edge.
     *
     * @return Value of the destination vertex of the edge.
     */
    int32_t get_destination() const noexcept { return m_destination; };

    /**
     * @brief Get the previous source vertex of the edge.
     *
     * @return Reference to the previous source vertex of the edge.
     */
    int32_t get_prev_source() const noexcept { return m_prev_source; };

    /**
     * @brief Get the previous destination vertex of the edge.
     *
     * @return Reference to the previous destination vertex of the edge.
     */
    int32_t get_prev_destination() const noexcept { return m_prev_destination; };

    /**
     * @brief Compare this edge with another edge based on weight.
     *
     * @param t_lhs The edge to compare with.
     * @return True if this edge's weight is greater than the other edge's weight.
     */
    bool operator>(const Edge& t_lhs) const noexcept { return m_weight > t_lhs.get_weight(); }

private:
    double m_weight {}; ///> Weight of the edge.
    int32_t m_source {}; ///> Source vertex of the edge.
    int32_t m_destination {}; ///> Destination vertex of the edge.
    int32_t m_prev_source {}; ///> Previous source for this edge
    int32_t m_prev_destination {}; ///> Previous destination for this edge
};

/**
 * @struct PairHash
 * @brief Custom hash function for std::pair<int32_t, int32_t>.
 *
 * This struct provides a custom hash function for pairs of integers, which is used in std::unordered_set.
 */
struct PairHash {
    /**
     * @brief Hash function operator.
     *
     * @param p A constant reference to a pair of int32_t.
     * @return A size_t representing the hash value of the input pair.
     */
    int32_t operator()(const std::pair<int32_t, int32_t>& p) const
    {
        auto b = std::hash<int32_t> {}(p.first);
        auto a = std::hash<int32_t> {}(p.second);
        return (a + b - 2) * (a + b - 1) / 2 + a;
    }
};

/**
 * @brief Generates an ordered pair from two integers.
 *
 * @param t_a First integer.
 * @param t_b Second integer.
 * @return An ordered pair where the first element is the minimum of t_a and t_b, and the second element is the maximum.
 */
inline std::pair<int32_t, int32_t> ordered_pair(int32_t t_a, int32_t t_b) noexcept { return std::make_pair(std::min(t_a, t_b), std::max(t_a, t_b)); };

/**
 * @struct InGraph
 * @brief Structure representing a graph.
 *
 * The Graph consists of nodes which are connected by edges. Each node is
 * represented by an integer identifier and stores a list of edges. Additionally,
 * some nodes are designated as terminal nodes.
 */
struct InGraph {
    std::unordered_map<int32_t, std::vector<Edge>> nodes {}; ///> Map of node IDs to their corresponding list of edges.
    std::unordered_map<std::pair<int32_t, int32_t>, double, PairHash> map_edge_weight {}; ///> Map of edges to their weights in graph.
    std::unordered_set<int32_t> terminal_nodes {}; ///> List of terminal node IDs.
};

/**
 * @typedef OutGraph
 * @brief Output graph. Contains the all mst edges corrsponding to input graph and total weight.
 */
using OutGraph = std::pair<std::vector<std::pair<int32_t, int32_t>>, double>;

#endif