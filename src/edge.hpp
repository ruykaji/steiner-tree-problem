#ifndef __EDGE_HPP__
#define __EDGE_HPP__

#include <cstdint>

/**
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
    explicit Edge(double t_weight, int32_t t_source, int32_t t_destination)
        : m_weight(t_weight)
        , m_source(t_source)
        , m_destination(t_destination) {};

    ~Edge() = default;

    /**
     * @brief Get the weight of the edge.
     *
     * @return Reference to the weight of the edge.
     */
    inline constexpr double get_weight() const { return m_weight; };

    /**
     * @brief Get the source vertex of the edge.
     *
     * @return Reference to the source vertex of the edge.
     */
    inline constexpr int32_t get_source() const { return m_source; };

    /**
     * @brief Get the destination vertex of the edge.
     *
     * @return Reference to the destination vertex of the edge.
     */
    inline constexpr int32_t get_destination() const { return m_destination; };

    /**
     * @brief Compare this edge with another edge based on weight.
     *
     * @param t_lhs The edge to compare with.
     * @return True if this edge's weight is greater than the other edge's weight.
     */
    inline constexpr bool operator>(const Edge& t_lhs) const { return m_weight > t_lhs.get_weight(); }

private:
    // Weight of the edge.
    double m_weight;

    // Source vertex of the edge.
    int32_t m_source;

    // Destination vertex of the edge.
    int32_t m_destination;
};

#endif