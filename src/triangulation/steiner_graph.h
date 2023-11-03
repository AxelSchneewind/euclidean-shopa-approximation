#pragma once

#include "../graph/adjacency_list.h"
#include "../graph/base_types.h"
#include "../graph/geometry.h"

#include <algorithm>
#include <cassert>
#include <span>
#include <vector>

struct subdivision_edge_info {
    // relative position of the points with highest distance to other edges
    float mid_position;
    // max distance of a point on this edge to other edges
    float mid_dist;

    // number of steiner points on this edge (counting the source and destination node)
    int node_count;
};

struct steiner_node_id {
    edge_id_t edge;
    int steiner_index;

    bool operator>=(const steiner_node_id &other) const {
        return edge >= other.edge || steiner_index >= other.steiner_index;
    }

    bool operator==(const steiner_node_id &other) const {
        return edge == other.edge && steiner_index == other.steiner_index;
    }
};

template<>
struct std::hash<steiner_node_id> {
    std::size_t operator()(const steiner_node_id &s) const noexcept;
};

std::size_t std::hash<steiner_node_id>::operator()(const steiner_node_id &s) const noexcept {
    std::size_t h1 = std::hash<edge_id_t>{}(s.edge);
    std::size_t h2 = std::hash<int>{}(s.steiner_index);
    return h1 ^ (h2 << 1);
}


struct steiner_edge_id {
    steiner_node_id source;
    steiner_node_id destination;
};

// class that stores edges (topology) of an adjacency list
class steiner_graph {
public:
    using node_id_type = steiner_node_id;
    using edge_id_type = steiner_edge_id;

    using node_info_type = node_t;
    using edge_info_type = edge_t;

    using triangle_node_id_t = node_id_t;
    using triangle_edge_id_t = edge_id_t;

    using triangle_node_info_t = node_t;
    using triangle_edge_info_t = edge_t;

    using adjacency_list_type = adjacency_list<triangle_edge_info_t>;

    struct node_id_iterator_type : public std::iterator<std::forward_iterator_tag, node_id_type> {
        node_id_iterator_type(const steiner_graph *__graph, node_id_type __current, node_id_type __max) : graph(
                __graph),
                                                                                                          current(__current),
                                                                                                          max(__max) {}

    private:
        const steiner_graph *graph;
        node_id_type current;
        node_id_type max;
    public:
        node_id_iterator_type &begin() { return *this; };

        node_id_iterator_type end() { return {graph, max, max}; };

        bool operator==(node_id_iterator_type __other) {
            return current.edge == __other.current.edge && current.steiner_index == __other.current.steiner_index;
        }

        bool operator!=(node_id_iterator_type __other) {
            return current.edge != __other.current.edge || current.steiner_index != __other.current.steiner_index;
        }

        node_id_iterator_type &operator++();

        node_id_type &operator*() { return current; }
    };

    steiner_graph(std::vector<steiner_graph::node_info_type> &&__triangulation_nodes,
                  adjacency_list<steiner_graph::triangle_edge_info_t> &&__triangulation_edges,
                  adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>> &&__triangles,
                  adjacency_list<subdivision_edge_info> &&__steiner_info);

    static steiner_graph make_graph(std::vector<steiner_graph::node_info_type> &&__triangulation_nodes,
                                    steiner_graph::adjacency_list_type &&__triangulation_edges);

private:
    size_t _M_node_count;
    size_t _M_edge_count;


    // store triangulation here
    std::vector<node_info_type> triangulation_nodes;
    adjacency_list<triangle_edge_info_t> triangulation;

    // store subdivision information here
    adjacency_list<subdivision_edge_info> steiner_info;

    // for each edge, store the id of the 2 nodes that make up the adjacent triangles
    adjacency_list<std::array<triangle_node_id_t, 2>> third_point;

    // returns the ids of the edges which are part of the 2 triangles bordering the given edge
    std::array<triangle_edge_id_t, 4> triangle_edges(const triangle_edge_id_t &__edge) const;

public:
    size_t node_count() const { return _M_node_count; }

    size_t edge_count() const { return _M_edge_count; }

    std::span<node_info_type, std::dynamic_extent> nodes() const { throw; /* TODO */};

    node_id_iterator_type node_ids() const;;

    node_info_type node(const node_id_type &__id) const;

    node_id_type source(const edge_id_type &__id) const;

    node_id_type destination(const edge_id_type &__id) const;

    edge_info_type edge(const edge_id_type &__id) const;

    edge_id_type edge_id(const node_id_type &src, const node_id_type &dest) const;

    bool has_edge(const node_id_type &src, const node_id_type &dest) const {
        return edge_id(src, dest).source.edge != NO_EDGE_ID; /*ToDO*/ };

    std::vector<internal_adjacency_list_edge<node_id_type, edge_info_type>>
    outgoing_edges(const node_id_type &__node_id) const;

    static adjacency_list<subdivision_edge_info>
    make_steiner_info(const adjacency_list<steiner_graph::triangle_edge_info_t> &triangulation,
                      const std::vector<steiner_graph::triangle_node_info_t> &nodes,
                      const adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>> &third_point,
                      float __epsilon);
};

steiner_graph::node_id_iterator_type &steiner_graph::node_id_iterator_type::operator++() {
    current.steiner_index++;
    if (current.steiner_index >= graph->steiner_info.edge(current.edge).node_count) {
        current.steiner_index = 0;
        current.edge++;
    }

    return *this;
}

//TODO
static_assert(Topology<steiner_graph>);
static_assert(RoutableGraph<steiner_graph>);
