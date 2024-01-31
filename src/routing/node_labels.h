#pragma once

#include "dijkstra_concepts.h"

#include <vector>

template<RoutableGraph G, typename Label>
class node_labels {
private:
    using node_id_type = G::node_id_type;
    using distance_type = G::distance_type;

    G const&_graph;

    std::vector<node_id_type> _touched;

    std::vector<Label> _labels;
    std::vector<bool> _node_labelled;

public:
    using label_type = Label;

    static constexpr size_t SIZE_PER_NODE = sizeof(Label) + 1;
    static constexpr size_t SIZE_PER_EDGE = 0;

    explicit node_labels(G const&d);

    ~node_labels() = default;

    node_labels(node_labels&&) noexcept = default;

    node_labels(G const& g, node_labels&& other) noexcept : _graph(g), _touched(std::move(other._touched)), _labels(std::move(other._labels)), _node_labelled(std::move(other._node_labelled)) {};

    node_labels& operator=(node_labels&&) noexcept = default;

    // init for given query
    void init(node_id_type start_node, node_id_type target_node);

    bool reached(node_id_type node) const;

    Label get(node_id_type node) const;
    Label& at(node_id_type node);
    Label const& at(node_id_type node) const;

    std::span<const node_id_type> all_visited() const;

    void label(node_id_type const&node, Label const&label);
};

template<RoutableGraph G, typename Label>
Label &node_labels<G, Label>::at(node_id_type node) {
    return _labels[node];
}

template<RoutableGraph G, typename Label>
Label const&node_labels<G, Label>::at(node_id_type node) const {
    return _labels[node];
}
