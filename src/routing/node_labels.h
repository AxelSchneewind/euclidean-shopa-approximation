#pragma once

#include "dijkstra_concepts.h"

#include <vector>

template<RoutableGraph G, typename Label>
class node_labels {
private:
    using node_id_type = G::node_id_type;
    using distance_type = G::distance_type;

    G const& _M_graph;

    std::vector<node_id_type> _M_touched;

    std::vector<Label> _M_labels;
    std::vector<bool> _M_node_labelled; // TODO remove

public:
    using label_type = Label;

    static constexpr size_t SIZE_PER_NODE = sizeof(Label) + 1;
    static constexpr size_t SIZE_PER_EDGE = 0;

    explicit node_labels(G const& d);

    // init for given query
    void init(node_id_type __start_node, node_id_type __target_node);

    bool reached(node_id_type __node) const;

    Label get(node_id_type __node) const;

    std::span<const node_id_type> all_visited() const;

    void label(node_id_type __node, Label __label);
};
