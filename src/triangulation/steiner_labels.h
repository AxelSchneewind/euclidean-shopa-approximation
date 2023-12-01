#pragma once

#include "../routing/dijkstra_concepts.h"
#include "node_info_array.h"
#include "compact_node_info_container.h"

#include <functional>

template<typename Outer, typename Inner>
class nested_iterator {
private:
    Outer outer;
    Outer outer_end;
    Inner inner;
    std::function<Inner(typename Outer::value_type)> iterator_retriever;
public:

    nested_iterator(Outer iterator, Outer outer_end, std::function<Inner(typename Outer::value_type)> retriever)
            : outer{iterator},
              outer_end{outer_end},
              inner{retriever(*outer)},
              iterator_retriever{std::move(retriever)} {};


    nested_iterator operator++() {
        auto result = *this;
        ++inner;
        if (inner == inner.end()) {
            outer++;
            inner = iterator_retriever(*outer);
        }
        return result;
    }

    nested_iterator &operator++(int) {
        ++inner;
        if (inner == inner.end()) {
            outer++;
            inner = iterator_retriever(*outer);
        }
        return *this;
    }

    struct end_type {
    };

    auto begin() { return *this; }

    end_type end() { return {}; }

    bool operator==(const end_type &) const { return outer == outer_end; }

    auto operator*() {
        return *inner;
    }
};


template<RoutableGraph G, typename Label>
class steiner_labels {
public:
    using label_type = Label;

    using label_iterator_type = nested_iterator<typename std::vector<steiner_graph::triangle_edge_id_type>::const_iterator, steiner_graph::node_id_iterator_type>;
    // static_assert(std::ranges::forward_range<label_iterator_type>);

private:
    using node_id_type = G::node_id_type;
    using distance_type = G::distance_type;

    G const &_M_graph;

    std::vector<typename G::triangle_edge_id_type> _M_touched;
    node_info_array<typename G::triangle_edge_id_type, unsigned short, label_type> _M_labels;
    // compact_node_info_container<typename G::triangle_edge_id_type, unsigned short, char, label_type> _M_labels;

public:
    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(std::unique_ptr<std::vector<Label>>);

    steiner_labels(G const &__graph);

    // init for given query
    void init(node_id_type __start_node, node_id_type __target_node);

    bool reached(node_id_type __node) const;

    Label get(node_id_type __node) const;

    label_iterator_type all_visited() const;

    void label(node_id_type __node, Label __label);
};
