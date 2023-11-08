#pragma once

#include "../graph/adjacency_list.h"
#include "../graph/base_types.h"
#include "../graph/geometry.h"
#include "polyhedron_impl.h"

#include <algorithm>
#include <cassert>
#include <span>
#include <vector>

struct steiner_node_id {
    edge_id_t edge;
    int steiner_index;

    steiner_node_id() : edge(none_value<edge_id_t>()), steiner_index(-1) {};

    constexpr steiner_node_id(edge_id_t __edge, int __steiner_index) : edge(__edge), steiner_index(__steiner_index) {}

    constexpr steiner_node_id(edge_id_t __edge) : edge(__edge), steiner_index(0) {}

    bool operator>=(const steiner_node_id &__other) const {
        return edge >= __other.edge || steiner_index >= __other.steiner_index;
    }

    bool operator>(const steiner_node_id &__other) const {
        return edge > __other.edge || (edge == __other.edge && steiner_index > __other.steiner_index);
    }

    bool operator==(const steiner_node_id &__other) const {
        return edge == __other.edge && steiner_index == __other.steiner_index;
    }
};

template<>
constexpr steiner_node_id none_value() { return {none_value<edge_id_t>(), none_value<int>()}; }

std::ostream &operator<<(std::ostream &output, steiner_node_id id) {
    return output << id.edge << ':' << id.steiner_index;
}

template<>
struct std::hash<steiner_node_id> {
    std::size_t operator()(const steiner_node_id &__s) const noexcept;
};

std::size_t std::hash<steiner_node_id>::operator()(const steiner_node_id &__s) const noexcept {
    std::size_t h1 = std::hash<edge_id_t>{}(__s.edge);
    std::size_t h2 = std::hash<int>{}(__s.steiner_index);
    return h1 ^ (h2 << 1);
}


struct steiner_edge_id {
    steiner_node_id source;
    steiner_node_id destination;


    bool operator>=(const steiner_edge_id &other) const {
        return source > other.source || (source == other.source && destination >= other.destination);
    }

    bool operator==(const steiner_edge_id &other) const {
        return source == other.source && destination == other.destination;
    }
};

template<>
constexpr steiner_edge_id none_value() { return {none_value<steiner_node_id>(), none_value<steiner_node_id>()}; }

std::ostream &operator<<(std::ostream &output, steiner_edge_id id) {
    return output << '(' << id.source << ',' << id.destination << ')';
}

template<>
struct std::hash<steiner_edge_id> {
    std::size_t operator()(const steiner_edge_id &s) const noexcept;
};

std::size_t std::hash<steiner_edge_id>::operator()(const steiner_edge_id &s) const noexcept {
    std::size_t h1 = std::hash<steiner_node_id>{}(s.source);
    std::size_t h2 = std::hash<steiner_node_id>{}(s.destination);
    return h1 ^ (h2 << 1);
}

// class that stores edges (topology) of an adjacency list
class steiner_graph {
public:
    using node_id_type = steiner_node_id;
    using edge_id_type = steiner_edge_id;

    using node_info_type = node_t;
    using edge_info_type = edge_t;

    using triangle_node_id_type = node_id_t;
    using triangle_edge_id_type = edge_id_t;

    using triangle_node_info_type = node_t;
    using triangle_edge_info_type = edge_t;

    using triangulation_type = adjacency_list<triangle_node_id_type, triangle_edge_info_type>;
    using adjacency_list_type = adjacency_list<triangle_node_id_type, triangle_edge_info_type>;

    using distance_type = distance_t;

    using path = path<node_id_type>;
    using subgraph = subgraph<node_id_type, edge_id_type>;

    using topology_type = steiner_graph;

    struct subdivision_edge_info {
        // relative position of the points with the highest distance to other edges
        float mid_position;
        // maximum distance of a point on this edge to other edges, relative to the length of this edge
        float mid_dist;

        // r(v), relative to this edge
        float r;

        // number of steiner points on this edge (counting the source and middle node)
        int node_count;

        bool operator==(const subdivision_edge_info &__other) const = default;
    };

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

    steiner_graph(std::vector<node_info_type> &&__triangulation_nodes,
                  adjacency_list<triangle_node_id_type, triangle_edge_info_type> &&__triangulation_edges,
                  polyhedron<triangulation_type, 3> &&__triangles,
                  adjacency_list<triangle_node_id_type, subdivision_edge_info> &&__steiner_info, float __epsilon);

private:
    size_t _M_node_count;
    size_t _M_edge_count;

    // the epsilon value used for discretization
    float _M_epsilon;

    // store triangulation here
    std::vector<node_info_type> triangulation_nodes;
    triangulation_type triangulation;

    // store subdivision information here
    adjacency_list<triangle_node_id_type, subdivision_edge_info> _M_steiner_info;

    // for each edge, store the id of the 2 nodes that make up the adjacent triangles
    polyhedron<steiner_graph::triangulation_type, 3> _M_polyhedron;

    // returns the ids of the edges which are part of the 2 triangles bordering the given edge
    std::span<const triangle_edge_id_type, 10> triangle_edges(const triangle_edge_id_type &__edge) const;

    coordinate_t node_coordinates(node_id_type __id) const;

public:
    const triangulation_type &base_graph() const { return triangulation; }

    size_t node_count() const { return _M_node_count; }

    size_t edge_count() const { return _M_edge_count; }

    node_id_iterator_type node_ids() const;

    node_id_iterator_type node_ids(triangle_edge_id_type __edge) const;

    node_info_type node(node_id_type __id) const;

    static node_id_type source(edge_id_type __id);

    static node_id_type destination(edge_id_type __id);

    edge_info_type edge(edge_id_type __id) const;

    subdivision_edge_info steiner_info(const triangle_edge_id_type &__id) const;

    edge_id_type edge_id(node_id_type src, node_id_type dest) const;

    bool has_edge(node_id_type src, node_id_type dest) const;

    const steiner_graph &topology() const { return *this; }

    const steiner_graph &inverse_topology() const { return *this; }

    std::vector<internal_adjacency_list_edge<node_id_type, edge_info_type>>
    outgoing_edges(node_id_type __node_id) const;

    static adjacency_list<triangle_node_id_type, subdivision_edge_info>
    make_steiner_info(const adjacency_list<triangle_node_id_type, triangle_edge_info_type> &triangulation,
                      const std::vector<triangle_node_info_type> &nodes,
                      const polyhedron<triangulation_type, 3> &__polyhedron,
                      float __epsilon);


    distance_type path_length(const path &__route) const;;

    subgraph make_subgraph(const path &__route) const;;

    static steiner_graph make_graph(std::vector<steiner_graph::node_info_type> &&__triangulation_nodes,
                                    steiner_graph::triangulation_type &&__triangulation_edges,
                                    std::vector<std::array<triangle_node_id_type, 3>> &&faces);

};

steiner_graph::node_id_iterator_type &steiner_graph::node_id_iterator_type::operator++() {
    current.steiner_index++;
    if (current.steiner_index >= graph->_M_steiner_info.edge(current.edge).node_count) {
        current.steiner_index = 0;
        current.edge++;
    }

    return *this;
}

static_assert(Topology<steiner_graph>);
static_assert(RoutableGraph<steiner_graph>);
