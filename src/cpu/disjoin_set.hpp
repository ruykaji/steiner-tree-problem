#ifndef __CPU_DISJOIN_SET_HPP__
#define __CPU_DISJOIN_SET_HPP__

#include <cstdint>
#include <unordered_map>
#include <unordered_set>

class CpuDisjointSet {
public:
    CpuDisjointSet() = default;
    ~CpuDisjointSet() = default;

    void make_set(int32_t t_set)
    {
        if (m_parent.find(t_set) == m_parent.end()) {
            m_parent[t_set] = t_set;
            m_rank[t_set] = 0;
        }
    }

    int32_t find(int32_t t_k)
    {
        if (m_parent.find(t_k) == m_parent.end()) {
            return -1;
        }

        if (m_parent[t_k] != t_k) {
            m_parent[t_k] = find(m_parent[t_k]);
        }

        return m_parent[t_k];
    }

    void union_sets(int32_t t_a, int32_t t_b)
    {
        int32_t x = find(t_a);
        int32_t y = find(t_b);

        if (x == -1 || y == -1 || x == y) {
            return;
        }

        if (m_rank[x] < m_rank[y]) {
            m_parent[x] = y;
        } else if (m_rank[x] > m_rank[y]) {
            m_parent[y] = x;
        } else {
            m_parent[y] = x;
            ++m_rank[x];
        }
    }

    void clear()
    {
        m_parent.clear();
        m_rank.clear();
    }

private:
    std::unordered_map<int32_t, int32_t> m_parent {};
    std::unordered_map<int32_t, int32_t> m_rank {};
};

#endif