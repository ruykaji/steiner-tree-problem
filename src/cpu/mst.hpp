#ifndef __CPU_MST_HPP__
#define __CPU_MST_HPP__

#include <iostream>
#include <memory>
#include <numeric>
#include <queue>

#include "disjoin_set.hpp"
#include "graph.hpp"

/**
 * @class CpuMST
 * @brief Implements a Minimum Spanning Tree algorithm on a CPU.
 *
 * This class is designed to compute the Minimum Spanning Tree (MST) of a given input graph.
 */
class CpuMST {
public:
    CpuMST() = default;
    ~CpuMST() = default;

    /**
     * @brief Function call operator to execute the MST algorithm.
     *
     * @param t_input_graph The input graph on which MST is to be computed.
     * @return The minimum spanning tree of the input graph as an OutGraph object.
     */
    std::unique_ptr<OutGraph> operator()(std::unique_ptr<InGraph> t_input_graph)
    {
        if (t_input_graph->nodes.size() == 0) {
            throw std::runtime_error("Graph doesn't contain any nodes!");
        }

        if (t_input_graph->terminal_nodes.size() == 0) {
            throw std::runtime_error("Graph doesn't contain any terminal nodes!");
        }

        m_graph = std::move(t_input_graph);

        reset();
        initialize_terminals_and_queue();
        process_edges();

        return restore_mst();
    };

private:
    /**
     * @brief Initializes terminals and the priority queue for the MST algorithm.
     *
     * Sets up the source and length arrays for each node and populates
     * the priority queue with initial edges.
     */
    void reset() noexcept
    {
        m_edge_queue = std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>>();
        m_terminal_set.clear();
        m_mst_edges.clear();
        m_source.assign(m_graph->nodes.size(), -1);
        m_length.assign(m_graph->nodes.size(), std::numeric_limits<double>::max());
        m_prev.assign(m_graph->nodes.size(), -1);
    }

    /**
     * @brief Initializes terminals and the priority queue for the MST algorithm.
     *
     * Sets up the source and length arrays for each node and populates
     * the priority queue with initial edges.
     */
    void initialize_terminals_and_queue()
    {
        for (const auto& terminal : m_graph->terminal_nodes) {
            m_terminal_set.make_set(terminal);
            m_source[terminal - 1] = terminal;
            m_length[terminal - 1] = 0;

            auto range = m_graph->nodes.equal_range(terminal);

            for (const auto edge : m_graph->nodes[terminal]) {
                if (edge->get_source() == terminal) {
                    m_edge_queue.emplace(*edge);
                } else {
                    m_edge_queue.emplace(Edge(edge->get_weight(), edge->get_destination(), edge->get_source()));
                }
            }
        }
    }

    /**
     * @brief Processes edges in the priority queue to build the MST.
     *
     * Processes each edge in the priority queue based on certain conditions,
     * thus contributing to the construction of the MST.
     */
    void process_edges()
    {
        while (!m_terminal_set.is_one_set()) {
            Edge edge = m_edge_queue.top();
            m_edge_queue.pop();

            int32_t destination = edge.get_destination();
            int32_t source = edge.get_source();
            int32_t prev_source = edge.get_prev_source();
            double weight = edge.get_weight();

            if (m_source[destination - 1] == -1) {
                m_source[destination - 1] = source;
                m_length[destination - 1] = weight;
                m_prev[destination - 1] = prev_source;

                for (auto e : m_graph->nodes[destination]) {
                    int32_t local_destination = e->get_destination() != destination ? e->get_destination() : e->get_source();

                    if (m_source[local_destination - 1] == -1) {
                        m_edge_queue.emplace(Edge(e->get_weight() + weight, source, local_destination, destination, -1));
                    }
                }
            } else if (m_terminal_set.find(m_source[destination - 1]) != m_terminal_set.find(source)) {
                if (m_graph->terminal_nodes.find(destination) != m_graph->terminal_nodes.end()) {
                    m_terminal_set.union_sets(source, destination);
                    m_mst_edges.emplace_back(edge);
                } else {
                    m_edge_queue.emplace(Edge(m_length[destination - 1] + weight, source, m_source[destination - 1], prev_source, destination));
                }
            }
        }
    }

    /**
     * @brief Restores the minimum spanning tree from the computed edges.
     *
     * @return The minimum spanning tree as an OutGraph object.
     */
    std::unique_ptr<OutGraph> restore_mst()
    {
        std::unordered_set<std::pair<int32_t, int32_t>, PairHash> result_path {};

        for (const auto& edge : m_mst_edges) {
            int32_t source = edge.get_source();
            int32_t destination = edge.get_destination();
            int32_t prev_source = edge.get_prev_source();
            int32_t prev_destination = edge.get_prev_destination();
            std::pair<int32_t, int32_t> pair {};

            while (prev_source != -1 && m_prev[prev_source - 1] != -1) {
                pair = ordered_pair(m_prev[prev_source - 1], prev_source);
                prev_source = m_prev[prev_source - 1];
                result_path.insert(pair);
            }

            while (prev_destination != -1 && m_prev[prev_destination - 1] != -1) {
                pair = ordered_pair(prev_destination, m_prev[prev_destination - 1]);
                prev_destination = m_prev[prev_destination - 1];
                result_path.insert(pair);
            }

            if (prev_source == -1 && prev_destination == -1) {
                pair = ordered_pair(source, destination);
                result_path.insert(pair);
            } else {
                pair = prev_source != -1 ? ordered_pair(source, prev_source) : ordered_pair(source, edge.get_prev_destination());
                result_path.insert(pair);

                pair = prev_destination != -1 ? ordered_pair(prev_destination, destination) : ordered_pair(edge.get_prev_source(), destination);
                result_path.insert(pair);

                if (prev_source != -1 && prev_destination != -1) {
                    pair = ordered_pair(edge.get_prev_source(), edge.get_prev_destination());
                    result_path.insert(pair);
                }
            }
        }

        return std::make_unique<OutGraph>(std::vector<std::pair<int32_t, int32_t>>(result_path.begin(), result_path.end()));
    }

private:
    CpuDisjointSet m_terminal_set {}; ///< Disjoint set data structure to manage terminals.
    std::unique_ptr<InGraph> m_graph {}; ///< The input graph on which MST is computed.
    std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> m_edge_queue {}; ///< Priority queue for edges based on their weights.
    std::vector<int32_t> m_source {}; ///< Source vertices for the edges.
    std::vector<double> m_length {}; ///< Lengths or weights of the edges.
    std::vector<int32_t> m_prev {}; ///< Previous vertices in the path.
    std::vector<Edge> m_mst_edges {}; ///< Edges that are part of the MST.
};

#endif