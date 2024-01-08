#pragma once

#include "dijkstra_concepts.h"

#include "../graph/unidirectional_adjacency_list.h"

#include "../triangulation/compact_node_info_container.h"

#include <concepts>
#include <tuple>
#include <queue>
#include <map>

template<RoutableGraph Graph>
struct use_all_edges {
public:
    use_all_edges(Graph const &g) {}

    use_all_edges() = default;

    use_all_edges(use_all_edges &&) = default;

    constexpr bool operator()(Graph::node_id_type /*node*/,
                              internal_adjacency_list_edge<typename Graph::node_id_type, typename Graph::edge_info_type> /*via*/) {
        return true;
    };
};

template<RoutableGraph Graph>
struct use_upward_edges {
protected:
    Graph const &g;

public:
    use_upward_edges(Graph const &g) : g(g) {}

    use_upward_edges(use_upward_edges &&) = default;

    constexpr bool operator()(Graph::node_id_type node,
                              internal_adjacency_list_edge<typename Graph::node_id_type, typename Graph::edge_info_type> via) {
        return g.node(node).level <= g.node(via.destination).level;
    };
};

struct Default {
public:
    Default() = default;

    template<typename NodeCostPair>
    constexpr bool operator()(const NodeCostPair &n1, const NodeCostPair &n2) {
        return n1.distance > n2.distance;
    };
};

template<RoutableGraph Graph, typename NodeCostPair, typename Comp = Default>
class dijkstra_queue : protected std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp> {
private:
    std::size_t _max_size{0};

    // count the number of push operations since last cleanup (bounds the number of duplicates currently present)
    std::size_t counter;

    // when to perform cleanup (0 means no cleanup)
    static constexpr std::size_t max_queue_size = 0;
    static constexpr std::size_t max_allowed_duplicates = max_queue_size;

protected:
    using base_queue_type = std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>;

public:
    using value_type = NodeCostPair;

    dijkstra_queue(Graph const &__graph, Comp __comp = Comp{})
            : std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>(__comp), counter(0) {}

    void init(Graph::node_id_type __start_node, Graph::node_id_type __target_node) {
        while (!empty())
            pop();

        counter = 0;
        _max_size = 0;
    };

    void push(base_queue_type::value_type ncp) {
        assert(ncp.distance != infinity<decltype(ncp.distance)>);
        base_queue_type::push(ncp);

        if constexpr (max_allowed_duplicates > 0) {
            // assumes that counter is the number of duplicates currently inserted
            counter++;
            if (counter >= max_allowed_duplicates) {
                cleanup();
                counter = 0;
            }
        }
    }

    void push(Graph::node_id_type __node, Graph::node_id_type __predecessor, distance_t __dist) {
        NodeCostPair ncp(__node, __predecessor, __dist);
        push(ncp);
    }

    void push_range(std::span<NodeCostPair, std::dynamic_extent> __nodes) {
        for (auto ncp: __nodes)
            push(ncp);
    }

    bool empty() const { return base_queue_type::empty(); }

    void pop() {
        _max_size = std::max(_max_size, base_queue_type::size());
        return base_queue_type::pop();
    }

    NodeCostPair top() const {
        return base_queue_type::top();
    }

    /**
     * perform sweep along container and remove duplicates (i.e. for node cost pairs with identical node, the one with minimal distance is kept)
     */
    void cleanup() {
        auto &container = base_queue_type::c;
        static std::unordered_map<typename Graph::node_id_type, unsigned int> first_index;

        size_t from_index = 0;
        size_t to_index = container.size();
        for (; from_index < to_index; from_index++) {
            auto ncp = container[from_index];
            auto node = container[from_index].node;

            // swap other node_cost_pair to current position if distance is larger than at the first occurrence
            while (from_index < to_index && first_index.contains(node)) [[likely]] {
                // if current instance has higher distance, move last element of vector here
                if (ncp.distance >= container[first_index[node]].distance)
                    [[likely]]
                            container[from_index] = container[--to_index];
                else // if current instance has lower distance, swap with first occurrence, next iteration will get last element
                    container[first_index[node]] = container[from_index];

                ncp = container[from_index];
                node = container[from_index].node;
            }

            first_index[node] = from_index;
        }
        container.resize(to_index);
        first_index.clear();

        std::make_heap(container.begin(), container.end(), base_queue_type::comp);
    }

    size_t max_size() const { return _max_size; }
};


template<typename Info>
struct aggregate_node_cost_pair {
    steiner_graph::base_topology_type::edge_id_type aggregate_id;
    steiner_graph::distance_type distance;
    Info info;

    aggregate_node_cost_pair() = default;

    aggregate_node_cost_pair(steiner_graph::base_topology_type::edge_id_type aggregate_id,
                             steiner_graph::distance_type distance, Info info)
            : aggregate_id{aggregate_id}, distance{distance}, info{info} {};

    steiner_graph::distance_type value() const { return info.value(); }
};

template<>
struct aggregate_node_cost_pair<void> {
    steiner_graph::base_topology_type::edge_id_type aggregate_id;
    steiner_graph::distance_type distance;

    aggregate_node_cost_pair() = default;

    aggregate_node_cost_pair(steiner_graph::base_topology_type::edge_id_type aggregate_id,
                             steiner_graph::distance_type distance)
            : aggregate_id{aggregate_id}, distance{distance} {};

    template<typename Info>
    aggregate_node_cost_pair(steiner_graph::base_topology_type::edge_id_type aggregate_id,
                             steiner_graph::distance_type distance, Info info)
            : aggregate_node_cost_pair(aggregate_id, distance) {};

    steiner_graph::distance_type value() const { return distance; }
};
