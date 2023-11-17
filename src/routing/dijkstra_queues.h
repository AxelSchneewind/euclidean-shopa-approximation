#pragma once

#include "dijkstra_impl.h"
#include <concepts>
#include <tuple>


template<RoutableGraph Graph, typename Info = std::nullptr_t>
struct node_cost_pair {
    Graph::node_id_type node;
    Graph::node_id_type predecessor;
    distance_t distance;
    Info info;
};

template<RoutableGraph Graph>
struct node_cost_pair<Graph, std::nullptr_t> {
    Graph::node_id_type node;
    Graph::node_id_type predecessor;
    distance_t distance;
};


template<RoutableGraph Graph>
struct use_all_edges {
protected:
    std::shared_ptr<const Graph> g;

public:
    use_all_edges(std::shared_ptr<const Graph> g) : g(g) {}

    constexpr bool operator()(Graph::node_id_type node,
                              internal_adjacency_list_edge<typename Graph::node_id_type, typename Graph::edge_info_type> via) {
        return true;
    };
};

template<RoutableGraph Graph>
struct use_upward_edges {
protected:
    std::shared_ptr<const Graph> g;

public:
    use_upward_edges(std::shared_ptr<const Graph> g) : g(g) {}

    bool operator()(Graph::node_id_type node,
                    internal_adjacency_list_edge<typename Graph::node_id_type, typename Graph::edge_info_type> via) {
        return g->node(node).level <= g->node(via.destination).level;
    };
};

template<typename NodeCostPair>
struct Default {
public:
    Default() = default;

    bool operator()(const NodeCostPair &n1, const NodeCostPair &n2) {
        return n1.distance > n2.distance;
    };
};

template<typename NodeCostPair>
struct A_Star {
public:
    A_Star() {};

    bool operator()(const NodeCostPair &__n1, const NodeCostPair &__n2) {
        return __n1.info.value > __n2.info.value;
    };
};

template<RoutableGraph Graph, typename NodeCostPair, typename Comp>
class dijkstra_queue : protected std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp> {
protected:
public:
    using value_type = NodeCostPair;

    dijkstra_queue(std::shared_ptr<const Graph> __graph, Comp __comp = Comp{})
            : std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>(__comp) {}

    virtual void init(Graph::node_id_type __start_node, Graph::node_id_type __target_node) {
        while (!empty())
            pop();
    };

    virtual void push(Graph::node_id_type __node, Graph::node_id_type __predecessor, distance_t __dist) {
        NodeCostPair ncp(__node, __predecessor, __dist);
        std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>::push(ncp);
    }

    Comp &get_comp() { return this->std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>::comp; }

    bool empty() const { return this->std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>::empty(); }

    void pop() { return this->std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>::pop(); }

    const NodeCostPair &top() const {
        return this->std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>::top();
    }
};

template<typename Graph, typename NodeCostPair, typename Comp = A_Star<NodeCostPair>>
class a_star_queue : public dijkstra_queue<Graph, NodeCostPair, Comp> {
private:
    std::shared_ptr<const Graph> _M_graph;
    coordinate_t _M_target_coordinates;
    float additional_distance;

public:
    using value_type = NodeCostPair;

    a_star_queue(std::shared_ptr<const Graph> __graph, Comp __comp = Comp{})
            : dijkstra_queue<Graph, NodeCostPair, Comp>(__graph, __comp), _M_graph(__graph) {}

    void push(Graph::node_id_type __node, Graph::node_id_type __predecessor, distance_t __dist) {
        NodeCostPair ncp(__node, __predecessor, __dist);
        if (ncp.info.value == 0)
            ncp.info.value = __dist + distance(_M_target_coordinates, _M_graph->node(__node).coordinates);
        std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>::push(ncp);
    }

    void init(Graph::node_id_type /*__start_node*/, Graph::node_id_type __target_node) override {
        while (!dijkstra_queue<Graph, NodeCostPair, Comp>::empty())
            dijkstra_queue<Graph, NodeCostPair, Comp>::pop();

        _M_target_coordinates = _M_graph->node(__target_node).coordinates;
    }

    void push_range(std::span<NodeCostPair, std::dynamic_extent> __nodes) {
        static std::vector<coordinate_t> coordinates;

        // get coordinates
        coordinates.resize(__nodes.size());
        for (int i = 0; i < __nodes.size(); ++i) {
            coordinates[i] = _M_graph->node(__nodes[i].node).coordinates;
        }

        // can be vectorized
        for (int i = 0; i < __nodes.size(); ++i) {
            __nodes[i].info.value = __nodes[i].distance + distance(_M_target_coordinates, coordinates[i]) + additional_distance;
        }

        for (auto ncp : __nodes)
            std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>::push(ncp);
    }

    void set_target(coordinate_t coordinates, float additional) {
        _M_target_coordinates = coordinates;
        additional_distance = additional;
    }
};