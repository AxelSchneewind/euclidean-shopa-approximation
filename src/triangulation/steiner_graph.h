#pragma once

#include "../graph/subgraph.h"
#include "subdivision_table.h"
#include "subdivision_info.h"

template<typename EdgeId, typename IntraEdgeId>
struct steiner_node_id {
    using edge_id_type = EdgeId;
    using intra_edge_id_type = IntraEdgeId;

    edge_id_type edge;
    intra_edge_id_type steiner_index;

    steiner_node_id() : edge(none_value<edge_id_type>), steiner_index(none_value<intra_edge_id_type>) {};

    constexpr steiner_node_id(EdgeId edge, IntraEdgeId steiner_index) : edge(edge),
                                                                            steiner_index(steiner_index) {}

    constexpr steiner_node_id(EdgeId edge, IntraEdgeId steiner_index, IntraEdgeId /*node_count*/)
            : edge(edge), steiner_index(steiner_index) {}

    bool operator>=(const steiner_node_id &other) const {
        return edge >= other.edge || steiner_index >= other.steiner_index;
    }

    bool operator>(const steiner_node_id &other) const {
        return edge > other.edge || (edge == other.edge && steiner_index > other.steiner_index);
    }

    bool operator==(const steiner_node_id &other) const {
        return edge == other.edge && steiner_index == other.steiner_index;
    }
};

// template<typename E, typename I>
// struct std::formatter<steiner_node_id<E, I>> : std::formatter<std::string> {
//     auto format(steiner_node_id<E, I> p, format_context &ctx) const {
//         return formatter<string>::format(std::format("{}:{}", p.edge, p.steiner_index), ctx);
//     }
// };

template<typename E, typename I>
struct std::hash<steiner_node_id<E, I>> {
    std::size_t operator()(const steiner_node_id<E, I> &s) const noexcept;
};


template<typename E, typename I>
constexpr steiner_node_id<E,I> none_value<steiner_node_id<E, I>> = {none_value<E>, none_value<I>};

template<typename E, typename I>
std::ostream &operator<<(std::ostream &output, steiner_node_id<E, I> id);

template<typename NodeId>
struct steiner_edge_id {
    NodeId source;
    NodeId destination;

    bool operator>=(const steiner_edge_id &other) const {
        return source > other.source || (source == other.source && destination >= other.destination);
    }

    bool operator==(const steiner_edge_id &other) const {
        return source == other.source && destination == other.destination;
    }
};


template<typename N>
constexpr steiner_edge_id<N> none_value<steiner_edge_id<N>> = {none_value<N>,
                                                               none_value<N>};

template<typename N>
std::ostream &operator<<(std::ostream &output, steiner_edge_id<N> id);

template<typename N>
struct std::hash<steiner_edge_id<N>> {
    std::size_t operator()(const steiner_edge_id<N> &s) const noexcept;
};


// template<typename N>
// struct std::formatter<steiner_edge_id<N>> : std::formatter<std::string> {
//     auto format(steiner_edge_id<N> p, format_context &ctx) const {
//         return formatter<string>::format(std::format("({},{})", p.source, p.destination), ctx);
//     }
// };

/**
 * provides access to a virtual graph derived from a base graph
 */
class steiner_graph {
public:
    using triangle_node_id_type = node_id_t;
    using triangle_edge_id_type = edge_id_t;

    using node_info_type = node_t;
    using edge_info_type = edge_t;

    using triangle_node_info_type = node_t;
    using triangle_edge_info_type = void;

    using base_topology_type = adjacency_list<triangle_node_id_type, triangle_edge_info_type>;
    using adjacency_list_type = adjacency_list<triangle_node_id_type, triangle_edge_info_type>;

    using polyhedron_type = polyhedron<base_topology_type, 3>;
    using topology_type = steiner_graph;

    using subdivision_info_type = subdivision;

    using intra_edge_id_type = typename subdivision_info_type::steiner_index_type;

    using node_id_type = steiner_node_id<triangle_edge_id_type, intra_edge_id_type>;
    using edge_id_type = steiner_edge_id<node_id_type>;

    using distance_type = distance_t;

    using path_type = path<steiner_graph>;
    using subgraph_type = subgraph<steiner_graph>;


    // whether base nodes should have outgoing face crossing edges
    static constexpr bool face_crossing_from_base_nodes { true };

    struct node_id_iterator_type {
    private:
        const steiner_graph *_M_graph_ptr;
        node_id_type _M_current_node;
        node_id_type _M_last_node;

    public:
        node_id_iterator_type(const steiner_graph *graph, node_id_type current, node_id_type max)
                : _M_graph_ptr(graph),
                  _M_current_node(current),
                  _M_last_node(max) {

            while (_M_current_node.edge < _M_last_node.edge &&
                   _M_graph_ptr->base_graph().source(_M_current_node.edge) >=
                   _M_graph_ptr->base_graph().destination(_M_current_node.edge))
                _M_current_node.edge++;
        }

        node_id_iterator_type &begin() { return *this; };

        struct end_type {
        };

        end_type end() { return {}; };

        bool operator==(node_id_iterator_type other) const {
            return _M_current_node.edge == other._M_current_node.edge &&
                   _M_current_node.steiner_index == other._M_current_node.steiner_index;
        }

        bool operator==(end_type other) const {
            return _M_current_node.edge >= _M_last_node.edge;
        }

        bool operator!=(node_id_iterator_type other) const {
            return _M_current_node.edge != other._M_current_node.edge ||
                   _M_current_node.steiner_index != other._M_current_node.steiner_index;
        }

        bool operator!=(end_type other) const {
            return !operator==(other);
        }

        node_id_iterator_type operator++();

        node_id_iterator_type operator++(int);

        node_id_type &operator*() { return _M_current_node; }
    };

    steiner_graph(steiner_graph &&other) noexcept;

    steiner_graph(std::vector<node_info_type> &&triangulation_nodes,
                  adjacency_list<triangle_node_id_type, triangle_edge_info_type> &&triangulation_edges,
                  polyhedron<base_topology_type, 3> &&triangles,
                  subdivision_info_type &&table,
                  std::vector<bool> && is_base_node,
                  double epsilon);


    static constexpr size_t SIZE_PER_NODE =
            sizeof(node_info_type) + base_topology_type::SIZE_PER_NODE + polyhedron_type::SIZE_PER_NODE +
            subdivision_info_type::SIZE_PER_NODE;
    static constexpr size_t SIZE_PER_EDGE =
            base_topology_type::SIZE_PER_EDGE + polyhedron_type::SIZE_PER_EDGE + subdivision_info_type::SIZE_PER_EDGE;

private:
    size_t _M_node_count;
    size_t _M_edge_count;

    // the epsilon value used for discretization
    double _M_epsilon;

    // store triangulation here
    std::vector<node_info_type> _M_base_nodes;
    std::vector<bool> _M_is_boundary_node;

    base_topology_type _M_base_topology;

    // store subdivision table here
    subdivision_info_type _M_table;

    // for each edge, store the id of the 2 nodes that make up the adjacent triangles
    polyhedron_type _M_polyhedron;

    // computes the coordinates of a node with given id
    coordinate_t node_coordinates(node_id_type __id) const;

public:
    node_id_type from_base_node_id(base_topology_type::node_id_type __node) const;

    const subdivision_info_type &subdivision_info() const { return _M_table; }

    const base_topology_type &base_graph() const { return _M_base_topology; }

    const polyhedron_type &base_polyhedron() const { return _M_polyhedron; }

    size_t node_count() const { return _M_node_count; }

    size_t edge_count() const { return _M_edge_count; }

    size_t face_count() const { return _M_polyhedron.face_count(); }

    double epsilon() const { return _M_epsilon; }

    node_id_iterator_type node_ids() const;

    node_id_iterator_type node_ids(triangle_edge_id_type __edge) const;

    node_info_type node(node_id_type id) const;

    node_info_type node(triangle_node_id_type id) const;
    node_info_type& node(triangle_node_id_type id) { return _M_base_nodes[id]; };

    bool is_base_node(node_id_type id) const;
    bool is_boundary_node(triangle_node_id_type id) const;
    bool is_base_neighboring_node(node_id_type id) const { return id.steiner_index == 1 || id.steiner_index ==
                                                                                                  steiner_info(id.edge).node_count - 2;};
    bool is_boundary_edge(triangle_edge_id_type id) const;

    triangle_node_id_type base_node_id(node_id_type id) const;

    static node_id_type source(edge_id_type id);

    static node_id_type destination(edge_id_type id);

    edge_info_type edge(edge_id_type id) const;

    subdivision_info_type::subdivision_edge_info const& steiner_info(triangle_edge_id_type id) const;
    subdivision_info_type::subdivision_edge_info & steiner_info(triangle_edge_id_type id) ;

    edge_id_type edge_id(node_id_type src, node_id_type dest) const;

    bool has_edge(node_id_type src, node_id_type dest) const;

    const steiner_graph &topology() const { return *this; }

    const steiner_graph &inverse_topology() const { return *this; }

    std::span<internal_adjacency_list_edge<node_id_type, edge_info_type>>
    outgoing_edges(node_id_type node_id) const;

    std::span<internal_adjacency_list_edge<node_id_type, edge_info_type>>
    outgoing_edges(triangle_node_id_type base_node_id) const;

    std::span<internal_adjacency_list_edge<node_id_type, edge_info_type>>
    incoming_edges(node_id_type node_id) const { return outgoing_edges(node_id); };

    std::span<internal_adjacency_list_edge<node_id_type, edge_info_type>>
    incoming_edges(triangle_node_id_type base_node_id) const { return outgoing_edges(base_node_id); };

    distance_type path_length(const path_type &route) const;

    subgraph_type make_subgraph(const path_type &route) const;

    static steiner_graph make_graph(std::vector<steiner_graph::node_info_type> &&triangulation_nodes,
                                    steiner_graph::base_topology_type &&triangulation_edges,
                                    std::vector<std::array<triangle_node_id_type, 3>> &&faces, double epsilon);

    static steiner_graph make_graph(steiner_graph const& other, subgraph<base_topology_type> const& subgraph);

};

static_assert(Topology<steiner_graph>);
static_assert(RoutableGraph<steiner_graph>);
