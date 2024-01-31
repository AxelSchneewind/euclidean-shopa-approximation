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
            ++outer; ++index;
            while(outer != outer_end && !(*outer)) {
                ++outer; ++index;
            }

            if (outer != outer_end)
                inner = iterator_retriever(index);
        }
        return result;
    }

    nested_iterator &operator++(int) {
        ++inner;
        if (inner == inner.end()) {
            ++outer; ++index;
            while(outer != outer_end && !(*outer)) {
                ++outer; ++index;
            }

            if (outer != outer_end)
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

    // using labels_type = node_info_array<typename G::triangle_edge_id_type, typename G::intra_edge_id_type, label_type>;
    using labels_type = compact_node_info_container<typename G::triangle_edge_id_type, typename G::intra_edge_id_type, char, label_type>;

    G const &_graph;

    std::vector<bool> _edge_touched;

    std::vector<label_type> _base_labels;
    labels_type _labels;

public:
    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(std::unique_ptr<std::vector<Label>>);

    steiner_labels(G const &graph);

    ~steiner_labels() = default;

    steiner_labels(steiner_labels &&other) noexcept
        : _graph(other._graph)
        , _edge_touched(std::move(other._edge_touched))
        , _base_labels(std::move(other._base_labels))
        , _labels(std::move(other._labels)) {};

    steiner_labels(G const& graph, steiner_labels &&other) noexcept
        : _graph(graph)
        , _edge_touched(std::move(other._edge_touched))
        , _base_labels(std::move(other._base_labels))
        , _labels(std::move(other._labels)) {};


    steiner_labels &operator=(steiner_labels &&) noexcept = default;

    // init for given query
    void init(node_id_type start_node, node_id_type target_node);

    Label get(node_id_type node) const;

    label_iterator_type all_visited() const;

    void label(node_id_type const& node, Label const& label);
};
