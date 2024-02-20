#pragma once

#include "../routing/dijkstra_concepts.h"
#include "compact_node_info_container.h"
#include "fast_map.h"

#include <functional>
#include <vector>

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

private:
    using node_id_type = typename G::node_id_type;
    using edge_id_type = typename G::triangle_edge_id_type;
    using intra_edge_id_type = typename G::triangle_edge_id_type;
    using distance_type = typename G::distance_type;

private:
    using labels_type = fast_map<edge_id_type, intra_edge_id_type, label_type>;

    G const &_graph;

    std::vector<bool> _edge_touched;

    std::vector<label_type> _base_labels;
    labels_type _labels;

public:
    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(std::unique_ptr<std::vector<Label>>);

    steiner_labels(G const &graph);

    ~steiner_labels() = default;

    steiner_labels(steiner_labels &&other) noexcept;;

    steiner_labels(G const& graph, steiner_labels &&other) noexcept;;


    steiner_labels &operator=(steiner_labels &&) noexcept = default;

    // init for given query
    [[gnu::cold]]
    void init(node_id_type start_node, node_id_type target_node);

    [[gnu::hot]]
    Label& at(node_id_type node);
    [[gnu::hot]]
    Label at(node_id_type node) const;

    [[gnu::cold]]
    label_iterator_type all_visited() const;

    [[gnu::hot]]
    void label(node_id_type const& node, Label const& label);
};
