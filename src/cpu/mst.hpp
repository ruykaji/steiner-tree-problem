#ifndef __CPU_MST_HPP__
#define __CPU_MST_HPP__

#include <iostream>
#include <numeric>
#include <queue>

#include "disjoin_set.hpp"
#include "graph.hpp"

class CpuMST {
    struct PairHash {
        std::size_t operator()(const std::pair<int32_t, int32_t>& p) const { return std::hash<int32_t> {}(p.first) ^ std::hash<int32_t> {}(p.second); }
    };

public:
    CpuMST() = default;
    ~CpuMST() = default;

    OutGraph operator()(const InGraph& t_input_graph)
    {
        m_graph = std::move(t_input_graph);

        reset();
        initialize_terminals_and_queue();
        process_edges();
        return restore_mst();
    };

private:
    void reset() noexcept
    {
        m_terminal_set.clear();
        std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>>().swap(m_edge_queue);

        m_source.clear();
        m_length.clear();
        m_prev.clear();
        m_mst_edges.clear();
    }

    void initialize_terminals_and_queue()
    {
        m_source.resize(m_graph.nodes.size(), -1);
        m_prev.resize(m_graph.nodes.size(), -1);
        m_length.resize(m_graph.nodes.size(), std::numeric_limits<double>::max());

        for (const auto& terminal : m_graph.terminal_nodes) {
            m_terminal_set.make_set(terminal);
            m_source[terminal - 1] = terminal;
            m_length[terminal - 1] = 0;

            for (const auto& edge : m_graph.nodes[terminal]) {
                m_edge_queue.emplace(edge);
            }
        }
    }

    std::pair<int32_t, int32_t> ordered_pair(int32_t t_a, int32_t t_b) { return std::make_pair(std::min(t_a, t_b), std::max(t_a, t_b)); };

    void process_edges()
    {
        while (!m_edge_queue.empty()) {
            Edge edge = m_edge_queue.top();
            m_edge_queue.pop();

            int32_t destination = edge.get_destination();
            int32_t source = edge.get_source();
            int32_t prev_source = edge.get_prev_source();
            double weight = edge.get_weight();

            int32_t& source_of_destination = m_source[destination - 1];
            double& length_of_destination = m_length[destination - 1];

            if (source_of_destination == -1) {
                source_of_destination = source;
                length_of_destination = weight;
                m_prev[destination - 1] = prev_source;

                for (const auto& e : m_graph.nodes[destination]) {
                    if (m_source[e.get_destination() - 1] == -1) {
                        m_edge_queue.emplace(Edge(e.get_weight() + weight, source, e.get_destination(), destination, -1));
                    }
                }
            } else if (m_terminal_set.find(source_of_destination) != m_terminal_set.find(source)) {
                if (m_graph.terminal_nodes.find(destination) != m_graph.terminal_nodes.end()) {
                    m_terminal_set.union_sets(source, source_of_destination);
                    m_mst_edges.emplace_back(edge);
                } else {
                    m_edge_queue.emplace(Edge(length_of_destination + weight, source, source_of_destination, prev_source, destination));
                }
            }
        }
    }

    OutGraph restore_mst()
    {
        std::unordered_set<std::pair<int32_t, int32_t>, PairHash> result_path {};
        double mst_weight {};

        for (const auto& edge : m_mst_edges) {
            int32_t source = edge.get_source();
            int32_t destination = edge.get_destination();
            int32_t prev_source = edge.get_prev_source();
            int32_t prev_destination = edge.get_prev_destination();

            while (prev_source != -1 && m_prev[prev_source - 1] != -1) {
                result_path.insert(ordered_pair(m_prev[prev_source - 1], prev_source));
                prev_source = m_prev[prev_source - 1];
            }

            while (prev_destination != -1 && m_prev[prev_destination - 1] != -1) {
                result_path.insert(ordered_pair(prev_destination, m_prev[prev_destination - 1]));
                prev_destination = m_prev[prev_destination - 1];
            }

            if (prev_source == -1 && prev_destination == -1) {
                result_path.insert(ordered_pair(source, destination));
            } else {
                if (prev_source != -1) {
                    result_path.insert(ordered_pair(source, prev_source));
                } else {
                    result_path.insert(ordered_pair(source, edge.get_prev_destination()));
                }

                if (prev_destination != -1) {
                    result_path.insert(ordered_pair(prev_destination, destination));
                } else {
                    result_path.insert(ordered_pair(edge.get_prev_source(), destination));
                }

                if (prev_source != -1 && prev_destination != -1) {
                    result_path.insert(ordered_pair(edge.get_prev_source(), edge.get_prev_destination()));
                }
            }

            mst_weight += edge.get_weight();
        }

        return std::make_pair(std::vector<std::pair<int32_t, int32_t>>(result_path.begin(), result_path.end()), mst_weight);
    }

private:
    InGraph m_graph {};
    CpuDisjointSet m_terminal_set {};
    std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> m_edge_queue {};
    std::vector<int32_t> m_source {};
    std::vector<double> m_length {};
    std::vector<int32_t> m_prev {};
    std::vector<Edge> m_mst_edges {};
};

#endif