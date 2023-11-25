#pragma once

#include "../routing/dijkstra_concepts.h"


template<RoutableGraph G, typename Label>
class steiner_labels {
public:
    using label_type = Label;

private:
    using node_id_type = G::node_id_type;
    using distance_type = G::distance_type;

    G const& _M_graph;

    std::vector<typename G::triangle_edge_id_type> _M_touched;
    std::vector<std::unique_ptr<std::vector<Label>>> _M_labels;  // TODO

public:
    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(std::unique_ptr<std::vector<Label>>);

    steiner_labels(G const& __graph);

    // init for given query
    void init(node_id_type __start_node, node_id_type __target_node);

    bool reached(node_id_type __node) const;

    Label get(node_id_type __node) const;

    // TODO make iterator
    std::vector<node_id_type> all_visited() const;

    void label(node_id_type __node, Label __label);
};
