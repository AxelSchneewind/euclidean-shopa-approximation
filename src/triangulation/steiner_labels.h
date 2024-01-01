#pragma once

#include "../routing/dijkstra_concepts.h"
#include "node_info_array.h"
#include "compact_node_info_container.h"

#include <functional>

template<typename Outer, typename Inner>
class nested_iterator {
private:
    size_t index;
    Outer outer;
    Outer outer_end;
    Inner inner;
    std::function<Inner(size_t)> iterator_retriever;
public:

    nested_iterator(Outer iterator, Outer outer_end, std::function<Inner(std::size_t)> retriever)
            : outer{iterator},
              outer_end{outer_end},
              index{0},
              inner{retriever(0)},
              iterator_retriever{std::move(retriever)} {
        while(!(*outer)) {
            ++outer; ++index;
        }
        inner = iterator_retriever(index);
    };


    nested_iterator operator++() {
        auto result = *this;
        ++inner;
        if (inner == inner.end()) {
            while(!(*outer)) {
                ++outer; ++index;
            }
            inner = iterator_retriever(index);
        }
        return result;
    }

    nested_iterator &operator++(int) {
        ++inner;
        if (inner == inner.end()) {
            while(!(*outer)) {
                ++outer; ++index;
            }
            inner = iterator_retriever(index);
        }
        return *this;
    }

    struct end_type {
    };

    auto begin() { return *this; }

    end_type end() { return {}; }

    bool operator==(const end_type &) const { return outer == outer_end || outer >= outer_end; }

    auto operator*() {
        return *inner;
    }
};


template<RoutableGraph G, typename Label>
class steiner_labels {
public:
    using label_type = Label;

    using label_iterator_type = nested_iterator<typename std::vector<bool>::const_iterator, steiner_graph::node_id_iterator_type>;
    // static_assert(std::ranges::forward_range<label_iterator_type>);

private:
    using node_id_type = G::node_id_type;
    using distance_type = G::distance_type;

    using labels_type = node_info_array<typename G::triangle_edge_id_type, unsigned short, label_type>;
    // using labels_type = compact_node_info_container<typename G::triangle_edge_id_type, unsigned short, char, label_type>;

    G const &_M_graph;

    std::vector<bool> _M_edge_touched;

    std::vector<label_type> _M_base_labels;
    labels_type _M_labels;

public:
    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(std::unique_ptr<std::vector<Label>>);

    steiner_labels(G const &__graph);

    ~steiner_labels() = default;

    steiner_labels(steiner_labels &&) noexcept = default;

    steiner_labels &operator=(steiner_labels &&) noexcept = default;

    // init for given query
    void init(node_id_type __start_node, node_id_type __target_node);

    Label get(node_id_type __node) const;

    label_iterator_type all_visited() const;

    void label(node_id_type __node, Label __label);
};
