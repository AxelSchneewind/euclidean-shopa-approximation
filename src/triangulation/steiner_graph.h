#pragma once

#include "../graph/adjacency_list.h"
#include "../graph/base_types.h"
#include "../graph/geometry.h"
#include "polyhedron_impl.h"

#include <algorithm>
#include <cassert>
#include <span>
#include <vector>
#include <stdfloat>

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

/**
 * provides access to a virtual graph derived from a base graph
 */
class steiner_graph {
public:
    using node_id_type = steiner_node_id;
    using edge_id_type = steiner_edge_id;

    using node_info_type = node_t;
    using edge_info_type = edge_t;

    using triangle_node_id_type = node_id_t;
    using triangle_edge_id_type = edge_id_t;

    using triangle_node_info_type = node_t;
    using triangle_edge_info_type = std::nullptr_t;

    using base_topology_type = adjacency_list<triangle_node_id_type, triangle_edge_info_type>;
    using adjacency_list_type = adjacency_list<triangle_node_id_type, triangle_edge_info_type>;

    using distance_type = distance_t;

    using path = path<node_id_type>;
    using subgraph = subgraph<node_id_type, edge_id_type>;

    using topology_type = steiner_graph;

    struct subdivision_edge_info {
        // relative position of the points with the highest distance to other edges
        std::float16_t mid_position;
        // maximum distance of a point on this edge to other edges, relative to the length of this edge
        std::float16_t mid_dist;

        // r(v), relative to this edge
        std::float16_t r;

        // number of steiner points on this edge (counting the source and middle node)
        short node_count;

        bool operator==(const subdivision_edge_info &__other) const = default;
    };

    struct node_id_iterator_type {
    private:
        const steiner_graph *_M_graph_ptr;
        node_id_type _M_current_node;
        node_id_type _M_last_node;

    public:
        node_id_iterator_type(const steiner_graph *__graph, node_id_type __current, node_id_type __max)
                : _M_graph_ptr(__graph),
                  _M_current_node(__current),
                  _M_last_node(__max) {}

        node_id_iterator_type &begin() { return *this; };

        node_id_iterator_type end() { return {_M_graph_ptr, _M_last_node, _M_last_node}; };

        bool operator==(node_id_iterator_type __other) const {
            return _M_current_node.edge == __other._M_current_node.edge &&
                   _M_current_node.steiner_index == __other._M_current_node.steiner_index;
        }

        bool operator!=(node_id_iterator_type __other) const {
            return _M_current_node.edge != __other._M_current_node.edge ||
                   _M_current_node.steiner_index != __other._M_current_node.steiner_index;
        }

        node_id_iterator_type &operator++();

        node_id_type &operator*() { return _M_current_node; }
    };

    template<size_t BaseEdgeCount>
    struct edges_iterator_type {
    private:
        const steiner_graph *_M_graph_ptr;
        node_id_type src;
        short edge_index;
        short node_index;
        std::array<base_topology_type::edge_id_type, BaseEdgeCount> _M_edges;

    public:
        edges_iterator_type(const steiner_graph *__graph, node_id_type src,
                            std::initializer_list<std::array<base_topology_type::edge_id_type, BaseEdgeCount>> __edges)
                : _M_graph_ptr(__graph),
                  src(src),
                  _M_edges(__edges),
                  node_index(0),
                  edge_index(0) {}

        edges_iterator_type(const steiner_graph *__graph, node_id_type src,
                            std::array<base_topology_type::edge_id_type, BaseEdgeCount> __edges, short edge_index = 0,
                            short node_index = 0)
                : _M_graph_ptr(__graph),
                  src(src),
                  _M_edges(__edges),
                  node_index(node_index),
                  edge_index(edge_index) {}

        edges_iterator_type &begin() { return *this; };

        struct end_type {
        };

        end_type end() { return end_type{}; };

        bool operator==(edges_iterator_type __other) const {
            return _M_edges == __other._M_edges &&
                   edge_index == __other.edge_index && node_index == __other.node_index;
        }

        bool operator==(end_type __other) const {
            return edge_index == BaseEdgeCount;
        }

        bool operator!=(edges_iterator_type __other) const {
            return node_index != edge_index || edge_index != __other.edge_index || _M_edges.edge != __other._M_edges;
        }

        bool operator!=(end_type __other) const {
            return edge_index != BaseEdgeCount;
        }

        edges_iterator_type &operator++() {
            edges_iterator_type &result = *this;
            node_index++;

            while (is_none(_M_edges[edge_index]) || node_index >= _M_graph_ptr->steiner_info(_M_edges[edge_index]).node_count) {
                node_index = 0;
                edge_index++;
            }

            return result;
        };

        edges_iterator_type &operator++(int) {
            node_index++;

            while (is_none(_M_edges[edge_index]) || node_index >= _M_graph_ptr->steiner_info(_M_edges[edge_index]).node_count) {
                node_index = 0;
                edge_index++;
            }

            return *this;
        };

        internal_adjacency_list_edge<node_id_type, edge_info_type> operator*() {
            node_id_type dest_id = {_M_edges[edge_index], node_index};
            edge_id_type id = { src, dest_id };
            return {dest_id, _M_graph_ptr->edge(id)};
        }
    };


    steiner_graph(std::vector<node_info_type> &&__triangulation_nodes,
                  adjacency_list<triangle_node_id_type, triangle_edge_info_type> &&__triangulation_edges,
                  polyhedron<base_topology_type, 3> &&__triangles,
                  std::vector<subdivision_edge_info> &&__steiner_info, float __epsilon);


    static constexpr size_t SIZE_PER_NODE = sizeof(node_info_type) + base_topology_type::SIZE_PER_NODE;
    static constexpr size_t SIZE_PER_EDGE =
            sizeof(subdivision_edge_info) + polyhedron<base_topology_type, 3>::SIZE_PER_EDGE;

private:
    size_t _M_node_count;
    size_t _M_edge_count;

    // the epsilon value used for discretization
    float _M_epsilon;

    // store triangulation here
    std::vector<node_info_type> _M_base_nodes;
    base_topology_type _M_base_topology;

    // vector for storing edges that are queried via incoming_edges() or outgoing_edges()
    static inline std::vector<internal_adjacency_list_edge<node_id_type, edge_info_type>> _temp_edges;

    // store subdivision information here
    std::vector<subdivision_edge_info> _M_steiner_info;

    // for each edge, store the id of the 2 nodes that make up the adjacent triangles
    polyhedron<steiner_graph::base_topology_type, 3> _M_polyhedron;

    // computes the coordinates of a node with given id
    coordinate_t node_coordinates(node_id_type __id) const;

public:
    const base_topology_type &base_graph() const { return _M_base_topology; }

    size_t node_count() const { return _M_node_count; }

    size_t edge_count() const { return _M_edge_count; }

    node_id_iterator_type node_ids() const;

    node_id_iterator_type node_ids(triangle_edge_id_type __edge) const;

    node_info_type node(node_id_type __id) const;

    node_info_type node(triangle_node_id_type __id) const;

    static node_id_type source(edge_id_type __id);

    static node_id_type destination(edge_id_type __id);

    edge_info_type edge(edge_id_type __id) const;

    subdivision_edge_info steiner_info(triangle_edge_id_type __id) const;

    edge_id_type edge_id(node_id_type __src, node_id_type __dest) const;

    bool has_edge(node_id_type __src, node_id_type __dest) const;

    const steiner_graph &topology() const { return *this; }

    const steiner_graph &inverse_topology() const { return *this; }

    // std::span<internal_adjacency_list_edge<node_id_type, edge_info_type>>
    edges_iterator_type<10>
    outgoing_edges(node_id_type __node_id) const;

    // std::span<internal_adjacency_list_edge<node_id_type, edge_info_type>>
    edges_iterator_type<10>
    incoming_edges(node_id_type __node_id) const { return outgoing_edges(__node_id); };

    static std::vector<subdivision_edge_info>
    make_steiner_info(const adjacency_list<triangle_node_id_type, triangle_edge_info_type> &__triangulation,
                      const std::vector<triangle_node_info_type> &__nodes,
                      const polyhedron<base_topology_type, 3> &__polyhedron,
                      float __epsilon);


    distance_type path_length(const path &__route) const;;

    subgraph make_subgraph(const path &__route) const;;

    static steiner_graph make_graph(std::vector<steiner_graph::node_info_type> &&__triangulation_nodes,
                                    steiner_graph::base_topology_type &&__triangulation_edges,
                                    std::vector<std::array<triangle_node_id_type, 3>> &&__faces);

};

static_assert(Topology<steiner_graph>);
static_assert(RoutableGraph<steiner_graph>);
