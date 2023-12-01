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
protected:
    using base_queue_type = std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>;

    // count the number of push operations since last cleanup (bounds the number of duplicates currently present)
    int counter;

    // when to perform cleanup
    static const int max_queue_size = 32000000;
    static const int max_allowed_duplicates = max_queue_size;

    size_t pull_counter;
    size_t push_counter;
    size_t _max_size;

public:
    using value_type = NodeCostPair;

    dijkstra_queue(Graph const &__graph, Comp __comp = Comp{})
            : std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>(__comp)
            , counter(0)
            , pull_counter(0)
            , push_counter(0)
            , _max_size(0)
            {}

    void init(Graph::node_id_type __start_node, Graph::node_id_type __target_node) {
        while (!empty())
            pop();

        counter = 0;
    };

    void push(base_queue_type::value_type ncp) {
        assert(ncp.distance != infinity<decltype(ncp.distance)>);
        base_queue_type::push(ncp);

        counter++;
        push_counter++;
        // assumes that counter is the number of duplicates currently inserted
        if (counter >= max_allowed_duplicates) {
            cleanup();
            counter = 0;
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
        pull_counter++;
        return base_queue_type::pop();
    }

    const NodeCostPair &top() const {
        return base_queue_type::top();
    }

    /**
     * perform sweep along container and remove duplicates (i.e. for node cost pairs with identical node, the one with minimal distance is kept)
     */
    void cleanup() {
        auto &container = base_queue_type::c;
        static std::unordered_map<typename Graph::node_id_type, short> first_index;

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

    size_t push_count() const { return push_counter; }
    size_t pull_count() const { return pull_counter; }
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


// template<typename NodeCostPair, typename Comp>
// class dijkstra_queue<steiner_graph, NodeCostPair, Comp> {
// private:
//     struct edge_max_dist {
//         steiner_graph::base_topology_type::edge_id_type aggregate_id;
//         steiner_graph::distance_type distance;
//     };
//
//     struct edge_info {
//         steiner_graph::distance_type distance;
//         std::vector<NodeCostPair> nodes;
//     };
//     //struct aggregate_node_info {
//     //    steiner_graph::distance_type distance;
//     //};
//
//     steiner_graph const &_M_graph;
//
//     using info_type = typename NodeCostPair::info_type;
//
//     using edge_ncp_type = aggregate_node_cost_pair<info_type>;
//     using edge_queue_type = std::priority_queue<edge_ncp_type, std::vector<edge_ncp_type>, Comp>;
//     edge_queue_type edge_queue; // can contain duplicates
//
//     // store node cost pairs here
//     std::unordered_map<steiner_graph::base_topology_type::edge_id_type, edge_info> active_edge_info;
//     // node_aggregate_container<steiner_graph::base_topology_type::edge_id_type, unsigned short, aggregate_node_info, NodeCostPair> active_edge_info;
//
//     // keep track of maximum distances of each edge (to know when to discard their node info), does NOT contain duplicates
//     std::priority_queue<edge_max_dist, std::vector<edge_max_dist>, Default> active_edges;
//
//     // temporary copy of the current node cost pair
//     NodeCostPair current;
//
// #ifndef NDEBUG
//     // for debugging
//     int active_edges_size;
//     int edge_queue_size;
// #endif
//
//
//     void update_current() {
//         auto _edge_ncp = edge_queue.top();
//         // assert(edge_queue.empty() || active_edge_info.is_expanded(_edge_ncp.aggregate_id));
//         assert(edge_queue.empty() || active_edge_info.contains(_edge_ncp.aggregate_id));
//
//         auto edge_info = active_edge_info[_edge_ncp.aggregate_id];
//         auto nodes = edge_info.nodes;
//         // auto edge_info = active_edge_info.get_aggregate_info(_edge_ncp.aggregate_id);
//         //auto nodes = active_edge_info.get(_edge_ncp.aggregate_id);
//         assert(nodes.size() > 0);
//
//         // find node with minimal distance that has not been returned yet
//         NodeCostPair result = nodes[0];
//         for (auto node_cost_pair: nodes) [[likely]] {
//             if (node_cost_pair.distance >= edge_info.distance && node_cost_pair.distance < result.distance) {
//                 result = node_cost_pair;
//             }
//         }
//
//         assert (!is_none(result.node));
//
// #ifndef NDEBUG
//         active_edges_size = active_edges.size();
//         edge_queue_size = edge_queue.size();
// #endif
//
//         current = result;
//     }
//
// public:
//     using value_type = NodeCostPair;
//
//     dijkstra_queue(steiner_graph const &__graph, Comp comp = Comp{})
//             : _M_graph(__graph), edge_queue{}//,
//             // active_edge_info({infinity<steiner_graph::distance_type>}, none_value<NodeCostPair>)
//             {}
//
//     void pop() {
//         edge_ncp_type current_edge_ncp;
//
//         // find nodes with minimal and second minimal distance that has not been returned yet
//         NodeCostPair result = none_value<NodeCostPair>;
//         NodeCostPair second_result;
//
//         // loop until valid node cost pair found
//         while (is_none(result)) [[unlikely]] {
//             //while (!active_edge_info.is_expanded(edge_queue.top().aggregate_id))
//             while (!active_edge_info.contains(edge_queue.top().aggregate_id))
//                 [[unlikely]]
//                         edge_queue.pop();
//
//             // remove edges that are not relevant anymore
//             while (!active_edges.empty() && active_edges.top().aggregate_id != edge_queue.top().aggregate_id &&
//                    active_edges.top().distance < edge_queue.top().value()) [[unlikely]] {
//                 //assert(active_edge_info.is_expanded(active_edges.top().aggregate_id));
//                 assert(active_edge_info.contains(active_edges.top().aggregate_id));
//                 assert(active_edges.top().aggregate_id != edge_queue.top().aggregate_id);
//                 active_edge_info.erase(active_edges.top().aggregate_id);
//                 active_edges.pop();
//             }
//
//             current_edge_ncp = edge_queue.top();
//
//             auto &&edge_info = active_edge_info[current_edge_ncp.aggregate_id];
//             auto &&info_list = edge_info.nodes;
//             // auto &&edge_info = active_edge_info.get_aggregate_info(current_edge_ncp.aggregate_id);
//             // auto &&info_list = active_edge_info.get(current_edge_ncp.aggregate_id);
//
//             // search for minimal and second minimal distance
//             result = none_value<NodeCostPair>;
//             second_result = none_value<NodeCostPair>;
//             for (auto node_cost_pair: info_list) {
//                 if (node_cost_pair.distance >= edge_info.distance && node_cost_pair.distance < result.distance) [[likely]] {
//                     second_result = result;
//                     result = node_cost_pair;
//                 }
//             }
//
//             //  remove edge
//             edge_queue.pop();
//         }
//
//         // reinsert edge if there are still nodes left to label
//         if (!is_none(second_result)) {
//             assert(active_edge_info.contains(current_edge_ncp.aggregate_id));
//             assert(active_edge_info[current_edge_ncp.aggregate_id].nodes.size() > 0);
//             // assert(active_edge_info.is_expanded(current_edge_ncp.aggregate_id));
//             // assert (!is_none(active_edge_info.get_aggregate_info(current_edge_ncp.aggregate_id).distance));
//             //active_edge_info.put_aggregate_info(current_edge_ncp.aggregate_id, {second_result.distance});
//             active_edge_info[current_edge_ncp.aggregate_id].distance = second_result.distance;
//             edge_queue.emplace(current_edge_ncp.aggregate_id, second_result.distance, second_result.value());
//         }
//
//         current = result;
//
//
//         // assert(active_edges.empty() || active_edge_info.is_expanded(active_edges.top().aggregate_id));
//         // assert(edge_queue.empty() || active_edge_info.is_expanded(edge_queue.top().aggregate_id));
//         assert(active_edges.empty() || active_edge_info.contains(active_edges.top().aggregate_id));
//         assert(edge_queue.empty() || active_edge_info.contains(edge_queue.top().aggregate_id));
//     }
//
//     void push(value_type ncp) {
//         auto edge = ncp.node.edge;
//         auto steiner_index = ncp.node.steiner_index;
//
//         // edge has not been seen yet
//         // if (!active_edge_info.is_expanded(edge)) [[unlikely]] {
//         if (!active_edge_info.contains(edge)) [[unlikely]] {
//             // allocate list of node cost pairs for this edge
//             active_edge_info.emplace(edge, edge_info{ncp.distance, std::vector<NodeCostPair>(_M_graph.steiner_info(edge).node_count, none_value<NodeCostPair>)});
//             assert(_M_graph.steiner_info(edge).node_count > 0);
//             // active_edge_info.expand(edge, _M_graph.steiner_info(edge).node_count);
//
//             // assert(is_infinity(active_edge_info.get(edge, 0).distance));
//             assert(is_infinity(active_edge_info[edge].nodes[0].distance));
//
//
//             // insert edge into active_edges
//             auto src = _M_graph.base_graph().source(edge);
//             auto dest = _M_graph.base_graph().destination(edge);
//             distance_t length = distance(_M_graph.node(src).coordinates, _M_graph.node(dest).coordinates);
//             active_edges.push({edge, ncp.value() + 10.0F * length});    // TODO use node radius
//         }
//
//         // update stored node cost pair
// //      if (ncp.distance < active_edge_info.get(edge, steiner_index).distance) [[likely]] {
// //           active_edge_info.put(edge, steiner_index, ncp);
//         if (ncp.distance < active_edge_info[edge].nodes[steiner_index].distance) [[likely]] {
//             active_edge_info[edge].nodes[ncp.node.steiner_index] = ncp;
// //          if (ncp.distance < active_edge_info.get_aggregate_info(edge).distance) {
//             if (ncp.distance < active_edge_info[edge].distance) {
// //              active_edge_info.put_aggregate_info(edge, { ncp.distance });
//                 active_edge_info[edge].distance =  ncp.distance;
//             }
//
//             // insert into queue
//             if constexpr (typeid(info_type) != typeid(void))
//                 edge_queue.emplace(edge, ncp.distance, ncp.info);
//             else
//                 edge_queue.emplace(edge, ncp.distance);
//         }
//
//         // assert(active_edge_info.is_expanded(edge));
//         assert(active_edge_info.contains(edge));
//         // assert(active_edges.empty() || active_edge_info.is_expanded(active_edges.top().aggregate_id));
//         assert(active_edges.empty() || active_edge_info.contains(active_edges.top().aggregate_id));
//         // assert(active_edge_info.is_expanded(ncp.node.edge));
//         assert(active_edge_info.contains(ncp.node.edge));
//         // assert(edge_queue.empty() || active_edge_info.is_expanded(edge_queue.top().aggregate_id));
//         assert(edge_queue.empty() || active_edge_info.contains(edge_queue.top().aggregate_id));
//         // assert (active_edge_info.is_expanded(edge_queue.top().aggregate_id));
//         assert (active_edge_info.contains(edge_queue.top().aggregate_id));
//
//         update_current();
//
//         // assert(active_edges.empty() || active_edge_info.is_expanded(active_edges.top().aggregate_id));
//         // assert(edge_queue.empty() || active_edge_info.is_expanded(edge_queue.top().aggregate_id));
//         assert(active_edges.empty() || active_edge_info.contains(active_edges.top().aggregate_id));
//         assert(edge_queue.empty() || active_edge_info.contains(edge_queue.top().aggregate_id));
//     }
//
//
//     void push(steiner_graph::node_id_type __node, steiner_graph::node_id_type __predecessor, distance_t __dist) {
//         NodeCostPair ncp(__node, __predecessor, __dist);
//         push(ncp);
//     }
//
//     void init(steiner_graph::node_id_type /*__start_node*/, steiner_graph::node_id_type __target_node) {
//         while (!edge_queue.empty())
//             edge_queue.pop();
//         while (!active_edges.empty())
//             active_edges.pop();
//
//         active_edge_info.clear();
//     }
//
//     void push_range(std::span<NodeCostPair, std::dynamic_extent> __nodes) {
//         for (auto ncp: __nodes)
//             [[likely]]
//                     push(ncp);
//     }
//
//     bool empty() const { return edge_queue.empty(); }
//
//
//     NodeCostPair top() const {
//         return current;
//     }
// };
