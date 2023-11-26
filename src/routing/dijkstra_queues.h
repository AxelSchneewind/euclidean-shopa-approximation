#pragma once

#include <concepts>
#include <tuple>
#include <queue>
#include <map>
#include "../triangulation/frontier_labels.h"


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

template<typename NodeCostPair>
struct Default {
public:
    Default() = default;

    constexpr bool operator()(const NodeCostPair &n1, const NodeCostPair &n2) {
        return n1.distance > n2.distance;
    };
};

template<typename NodeCostPair>
struct A_Star {
public:
    A_Star() {};

    constexpr bool operator()(const NodeCostPair &__n1, const NodeCostPair &__n2) {
        return __n1.info.value > __n2.info.value;
    };
};

template<RoutableGraph Graph, typename NodeCostPair, typename Comp = Default<NodeCostPair>>
class dijkstra_queue : protected std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp> {
protected:
    using base_queue_type = std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>;

    // count the number of push operations since last cleanup (bounds the number of duplicates currently present)
    int counter;

    // when to perform cleanup
    static const int max_queue_size = 32000;
    static const int max_allowed_duplicates = max_queue_size;

public:
    using value_type = NodeCostPair;

    dijkstra_queue(Graph const &__graph, Comp __comp = Comp{})
            : std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>(__comp),
              counter(0) {}

    void init(Graph::node_id_type __start_node, Graph::node_id_type __target_node) {
        while (!empty())
            pop();

        counter = 0;
    };

    void push(base_queue_type::value_type ncp) {
        base_queue_type::push(ncp);

        counter++;
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

    void pop() { return base_queue_type::pop(); }

    const NodeCostPair &top() const {
        return base_queue_type::top();
    }

    /**
     * perform sweep along container and remove duplicates (i.e. for node cost pairs with identical node, the one with minimal distance is kept)
     */
    void cleanup() {
        auto &container = base_queue_type::c;
        static std::unordered_map<typename Graph::node_id_type, short> first_index;

        int from_index = 0;
        int to_index = container.size();
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
};


struct a_star_info {
    // value from a* heuristic (distance + minimal remaining distance)
    distance_t value;

    bool operator==(a_star_info const &) const = default;

    distance_t min_distance() const { return value; };
};

template<>
constexpr a_star_info none_value<a_star_info> = {infinity<distance_t>};


template<typename Graph, typename NodeCostPair, typename Comp = A_Star<NodeCostPair>>
class a_star_queue : public dijkstra_queue<Graph, NodeCostPair, Comp> {
private:
    using base_queue_type = dijkstra_queue<Graph, NodeCostPair, Comp>;
    Graph const &_M_graph;
    coordinate_t _M_target_coordinates;
    float additional_distance;

public:
    using value_type = NodeCostPair;

    a_star_queue(Graph const &__graph, Comp __comp = Comp{})
            : dijkstra_queue<Graph, NodeCostPair, Comp>(__graph, __comp), _M_graph(__graph) {}

    void push(Graph::node_id_type __node, Graph::node_id_type __predecessor, distance_t __dist) {
        NodeCostPair ncp(__node, __predecessor, __dist);
        ncp.info.value = __dist + distance(_M_target_coordinates, _M_graph.node(__node).coordinates);
        base_queue_type::push(ncp);
    }

    void init(Graph::node_id_type /*__start_node*/, Graph::node_id_type __target_node) {
        while (!dijkstra_queue<Graph, NodeCostPair, Comp>::empty())
            dijkstra_queue<Graph, NodeCostPair, Comp>::pop();

        _M_target_coordinates = _M_graph.node(__target_node).coordinates;
    }

    void push_range(std::span<NodeCostPair, std::dynamic_extent> __nodes) {
        static std::vector<coordinate_t> coordinates;

        // get coordinates
        coordinates.resize(__nodes.size());
        for (int i = 0; i < __nodes.size(); ++i) {
            coordinates[i] = _M_graph.node(__nodes[i].node).coordinates;
        }

        // can be vectorized
        for (int i = 0; i < __nodes.size(); ++i) {
            __nodes[i].info.value =
                    __nodes[i].distance + distance(_M_target_coordinates, coordinates[i]) + additional_distance;
        }

        for (auto ncp: __nodes)
            base_queue_type::push(ncp);
    }

    void set_target(coordinate_t coordinates, float additional) {
        _M_target_coordinates = coordinates;
        additional_distance = additional;
    }
};


// TODO: move specialization to dijkstra queue
template<typename NodeCostPair, typename Comp>
class a_star_queue<steiner_graph, NodeCostPair, Comp> {
private:
    struct edge_ncp {
        steiner_graph::base_topology_type::edge_id_type edge;
        steiner_graph::distance_type distance;
        a_star_info info;
    };

    struct edge_max_dist {
        steiner_graph::base_topology_type::edge_id_type edge;
        steiner_graph::distance_type distance;
    };

    struct edge_info {
        steiner_graph::distance_type distance;
        std::vector<NodeCostPair> nodes;
    };

    using base_queue_type = std::priority_queue<edge_ncp, std::vector<edge_ncp>, A_Star<edge_ncp>>;
    steiner_graph const &_M_graph;
    coordinate_t _M_target_coordinates;
    distance_t additional_distance;

    base_queue_type edge_queue; // can contain duplicates

    // store node cost pairs here
    std::unordered_map<steiner_graph::base_topology_type::edge_id_type, edge_info> active_edge_info;
    // keep track of maximum distances of each edge (to know when to discard their node info), does NOT contain duplicates
    std::priority_queue<edge_max_dist, std::vector<edge_max_dist>, Default<edge_max_dist>> active_edges;

    // temporary copy of the current node cost pair
    NodeCostPair current;

#ifndef NDEBUG
    // for debugging
    int active_edges_size;
    int edge_queue_size;
    int active_edge_info_size;
#endif
public:
    using value_type = NodeCostPair;

    a_star_queue(steiner_graph const &__graph)
            : _M_graph(__graph), edge_queue{} {}


    void pop() {
        edge_ncp current_edge_ncp;

        // find nodes with minimal and second minimal distance that has not been returned yet
        NodeCostPair result = none_value<NodeCostPair>;
        NodeCostPair second_result;

        while (is_none(result)) [[unlikely]] {
            current_edge_ncp = edge_queue.top();

            // remove edges that are not relevant anymore
            while (!active_edges.empty() && active_edges.top().edge != current_edge_ncp.edge &&
                   active_edges.top().distance < current_edge_ncp.info.value) [[unlikely]] {
                assert(active_edge_info.contains(active_edges.top().edge));
                assert(active_edges.top().edge != edge_queue.top().edge);
                active_edge_info.erase(active_edges.top().edge);
                active_edges.pop();
            }

            assert(active_edge_info.contains(current_edge_ncp.edge));
            auto &&edge_info = active_edge_info[current_edge_ncp.edge];

            // search for minimal and second minimal distance
            result = none_value<NodeCostPair>;
            second_result = none_value<NodeCostPair>;
            for (auto node_cost_pair: edge_info.nodes) {
                if (node_cost_pair.distance >= edge_info.distance &&
                    node_cost_pair.distance < result.distance) [[likely]] {
                    second_result = result;
                    result = node_cost_pair;
                }
            }

            //  remove edge
            edge_queue.pop();
        }

        // reinsert edge if there are still nodes left to label
        if (!is_none(second_result)) {
            assert(active_edge_info.contains(current_edge_ncp.edge));
            assert(active_edge_info[current_edge_ncp.edge].nodes.size() > 0);
            assert (!is_none(active_edge_info.at(current_edge_ncp.edge).distance));
            active_edge_info[current_edge_ncp.edge].distance = second_result.distance;
            edge_queue.push(edge_ncp{current_edge_ncp.edge, second_result.distance, second_result.info.value});
        } else {    // remove otherwise
            // active_edge_info.erase(current_edge_ncp.edge);
        }

        while (!active_edge_info.contains(edge_queue.top().edge))
            [[unlikely]]
                    edge_queue.pop();

        update_current();

        assert(active_edges.empty() || active_edge_info.contains(active_edges.top().edge));
        assert(edge_queue.empty() || active_edge_info.contains(edge_queue.top().edge));
        assert(active_edge_info.at(edge_queue.top().edge).nodes.size() > 0);

        assert(active_edges.empty() || active_edge_info.contains(active_edges.top().edge));
        assert(edge_queue.empty() || active_edge_info.contains(edge_queue.top().edge));
        assert(active_edge_info.at(edge_queue.top().edge).nodes.size() > 0);
    }

    void push(NodeCostPair ncp) {
        auto edge = ncp.node.edge;

        // edge has not been seen yet
        if (!active_edge_info.contains(edge)) [[unlikely]] {
            // allocate list of node cost pairs for this edge
            active_edge_info.insert(
                    std::make_pair(edge, edge_info{ncp.distance, std::vector<NodeCostPair>(
                            _M_graph.steiner_info(edge).node_count, none_value<NodeCostPair>)}));
            assert(is_infinity(active_edge_info.at(edge).nodes.at(0).distance));
            assert(active_edge_info.at(edge).nodes.size() >= _M_graph.steiner_info(edge).node_count);


            // insert edge into active_edges
            auto src = _M_graph.base_graph().source(edge);
            auto dest = _M_graph.base_graph().destination(edge);
            distance_t length = distance(_M_graph.node(src).coordinates, _M_graph.node(dest).coordinates);
            active_edges.push({edge, ncp.info.value + 2.0F * length});
        }

        // update stored node cost pair
        if (ncp.distance < active_edge_info[edge].nodes[ncp.node.steiner_index].distance) [[likely]] {
            active_edge_info[edge].nodes[ncp.node.steiner_index] = ncp;
            if (ncp.distance < active_edge_info[edge].distance) {
                active_edge_info[edge].distance = ncp.distance;
            }

            // insert into queue
            edge_queue.emplace(edge, ncp.distance, ncp.info);
        }

        assert(active_edge_info.contains(edge));
        assert(active_edges.empty() || active_edge_info.contains(active_edges.top().edge));
        assert(active_edge_info.contains(ncp.node.edge));
        assert(edge_queue.empty() || active_edge_info.contains(edge_queue.top().edge));
        assert(active_edge_info.at(edge).nodes.size() > 0);
        assert(active_edge_info.at(edge_queue.top().edge).nodes.size() > 0);

        // pop duplicates (why even necessary?)
        assert (active_edge_info.contains(edge_queue.top().edge));
        while (!active_edge_info.contains(edge_queue.top().edge))
            edge_queue.pop();

        update_current();

        assert(active_edges.empty() || active_edge_info.contains(active_edges.top().edge));
        assert(edge_queue.empty() || active_edge_info.contains(edge_queue.top().edge));
        assert(active_edge_info.at(edge).nodes.size() > 0);
        assert(active_edge_info.at(edge_queue.top().edge).nodes.size() > 0);
    }


    void push(steiner_graph::node_id_type __node, steiner_graph::node_id_type __predecessor, distance_t __dist) {
        NodeCostPair ncp(__node, __predecessor, __dist,
                         a_star_info{__dist + distance(_M_target_coordinates, _M_graph.node(__node).coordinates)});
        push(ncp);
    }

    void init(steiner_graph::node_id_type /*__start_node*/, steiner_graph::node_id_type __target_node) {
        while (!edge_queue.empty())
            edge_queue.pop();
        while (!active_edges.empty())
            active_edges.pop();

        active_edge_info.clear();

        _M_target_coordinates = _M_graph.node(__target_node).coordinates;
    }

    void push_range(std::span<NodeCostPair, std::dynamic_extent> __nodes) {
        static std::vector<coordinate_t> coordinates;

        // get coordinates
        coordinates.resize(__nodes.size());
        for (size_t i = 0; i < __nodes.size(); ++i) {
            coordinates[i] = _M_graph.node(__nodes[i].node).coordinates;
        }

        // can be vectorized
        for (size_t i = 0; i < __nodes.size(); ++i) {
            __nodes[i].info.value = __nodes[i].distance + 0.98 * distance(_M_target_coordinates, coordinates[i]);
        }

        for (auto ncp: __nodes)
            [[likely]]
                    push(ncp);
    }

    void set_target(coordinate_t coordinates, float additional) {
        _M_target_coordinates = coordinates;
        additional_distance = additional;
    }

    bool empty() const { return edge_queue.empty(); }

    void update_current() {
        assert(edge_queue.empty() || active_edge_info.contains(edge_queue.top().edge));

        auto _edge_ncp = edge_queue.top();
        auto edge_info = active_edge_info[_edge_ncp.edge];

        assert(edge_info.nodes.size() > 0);

        // find node with minimal distance that has not been returned yet
        NodeCostPair result = edge_info.nodes[0];
        for (auto node_cost_pair: edge_info.nodes) {
            if (node_cost_pair.distance >= edge_info.distance && node_cost_pair.distance < result.distance) {
                result = node_cost_pair;
            }
        }

        assert (!is_none(result.node));

#ifndef NDEBUG
        active_edges_size = active_edges.size();
        edge_queue_size = edge_queue.size();
        active_edge_info_size = active_edge_info.size();
#endif

        current = result;
    }

    NodeCostPair top() const {
        return current;
    }
};
