#ifndef __CPU_MST_HPP__
#define __CPU_MST_HPP__

#include <iostream>
#include <numeric>
#include <queue>

#include "disjoin_set.hpp"
#include "graph.hpp"

class CpuMST {
public:
    CpuMST() = default;
    ~CpuMST() = default;

    std::pair<std::vector<std::vector<int32_t>>, double> make(const Graph& t_input_graph)
    {
        m_graph = std::move(t_input_graph);

        reset();
        initialize_terminals_and_queue();

        return process_edges();
    };

private:
    void reset() noexcept
    {
        m_terminal_set.clear();
        std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>>().swap(m_edge_queue);

        m_source.clear();
        m_length.clear();
        m_prev.clear();
    }

    void initialize_terminals_and_queue()
    {
        m_source.resize(m_graph.nodes.size(), -1);
        m_prev.resize(m_graph.nodes.size(), -1);
        m_length.resize(m_graph.nodes.size(), std::numeric_limits<double>::max());

        for (auto terminal : m_graph.terminal_nodes) {
            m_terminal_set.make_set(terminal);
            m_source[terminal - 1] = terminal;
            m_length[terminal - 1] = 0;

            for (auto edge : m_graph.nodes[terminal]) {
                m_edge_queue.emplace(edge);
            }
        }
    }

    std::pair<std::vector<std::vector<int32_t>>, double> process_edges()
    {
        std::unordered_set<int32_t> mst_terminal_nodes {};
        std::vector<Edge> mst_edges {};

        while (mst_terminal_nodes != m_graph.terminal_nodes) {
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

                for (auto e : m_graph.nodes[destination]) {
                    if (m_source[e.get_destination() - 1] == -1) {
                        m_edge_queue.emplace(Edge(e.get_weight() + weight, source, e.get_destination(), destination, -1));
                    }
                }
            } else if (m_terminal_set.find(source_of_destination) != m_terminal_set.find(source)) {
                if (m_graph.terminal_nodes.find(destination) != m_graph.terminal_nodes.end()) {
                    m_terminal_set.union_sets(source, source_of_destination);

                    mst_terminal_nodes.insert(destination);
                    mst_terminal_nodes.insert(source);
                    mst_edges.push_back(edge);
                } else {
                    m_edge_queue.emplace(Edge(length_of_destination + weight, source, source_of_destination, prev_source, destination));
                }
            }
        }

        std::vector<std::vector<int32_t>> result_path {};
        double mst_weight {};

        for (auto edge : mst_edges) {
            std::vector<int32_t> path { edge.get_source(), edge.get_destination() };
            int32_t prev_source = edge.get_prev_source();
            int32_t prev_destination = edge.get_prev_destination();

            while (prev_source != -1 || prev_destination != -1) {
                if (prev_source != -1) {
                    path.insert(path.begin() + 1, prev_source);
                    prev_source = m_prev[prev_source - 1];
                }

                if (prev_destination != -1) {
                    path.insert(path.end() - 1, prev_destination);
                    prev_destination = m_prev[prev_destination - 1];
                }
            }

            result_path.emplace_back(path);
            mst_weight += edge.get_weight();
        }

        return std::make_pair(result_path, mst_weight);
    }

private:
    Graph m_graph {};
    CpuDisjointSet m_terminal_set {};
    std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> m_edge_queue {};
    std::vector<int32_t> m_source {};
    std::vector<double> m_length {};
    std::vector<int32_t> m_prev {};
};

#endif