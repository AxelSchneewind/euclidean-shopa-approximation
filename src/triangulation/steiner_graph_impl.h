#pragma once

#include "steiner_graph.h"

#include "subdivision_info_impl.h"
#include "polyhedron_impl.h"


#include "../util/set_minus.h"


std::size_t std::hash<steiner_node_id>::operator()(const steiner_node_id &__s) const noexcept {
    std::size_t h1 = std::hash<edge_id_t>{}(__s.edge);
    std::size_t h2 = std::hash<int>{}(__s.steiner_index);
    return h1 ^ (h2 << 1);
}

std::size_t std::hash<steiner_edge_id>::operator()(const steiner_edge_id &s) const noexcept {
    std::size_t h1 = std::hash<steiner_node_id>{}(s.source);
    std::size_t h2 = std::hash<steiner_node_id>{}(s.destination);
    return h1 ^ (h2 << 1);
}


std::ostream &operator<<(std::ostream &output, steiner_node_id id) {
    return output << id.edge << ':' << id.steiner_index;
}


std::ostream &operator<<(std::ostream &output, steiner_edge_id id) {
    return output << '(' << id.source << ',' << id.destination << ')';
}


void make_node_radii(
        const std::vector<steiner_graph::node_info_type> &__nodes,
        const steiner_graph::base_topology_type &__triangulation,
        const steiner_graph::polyhedron_type &__polyhedron,
        float __epsilon,
        std::vector<std::float16_t> &__out) {
    __out.clear();
    __out.resize(__nodes.size());

    std::vector<steiner_graph::triangle_edge_id_type> adjacent_edge_ids;
    std::vector<steiner_graph::triangle_edge_id_type> triangle_edge_ids;
    for (int node = 0; node < __triangulation.node_count(); ++node) {
        // get edges and triangles adjacent to node
        for (auto edge: __triangulation.outgoing_edges(node)) {
            auto edge_id = __triangulation.edge_id(node, edge.destination);
            auto inv_edge_id = __triangulation.edge_id(edge.destination, node);
            adjacent_edge_ids.push_back(edge_id);
            adjacent_edge_ids.push_back(inv_edge_id);

            for (auto triangle_edge: __polyhedron.edges(edge_id))
                triangle_edge_ids.push_back(triangle_edge);
        }

        std::sort(triangle_edge_ids.begin(), triangle_edge_ids.end());
        std::sort(adjacent_edge_ids.begin(), adjacent_edge_ids.end());

        // get edges that are only reachable via a triangle
        remove_duplicates_sorted(triangle_edge_ids);
        remove_duplicates_sorted(adjacent_edge_ids);
        int edge_count = set_minus_sorted<int>(triangle_edge_ids, adjacent_edge_ids, triangle_edge_ids);

        // get minimal value from triangle edges
        double dist = infinity<std::float16_t>;
        auto c3 = __nodes[node].coordinates;
        for (int i = 0; i < edge_count; i++) {
            auto edge_id = triangle_edge_ids[i];
            auto c1 = __nodes[__triangulation.source(edge_id)].coordinates;
            auto c2 = __nodes[__triangulation.destination(edge_id)].coordinates;
            if (c3 != c2 && c3 != c1 && c2 != c1)
                dist = std::min(dist, line_distance(c1, c2, c3));
        }
        dist = std::max(dist, 0.001);
        __out[node] = dist;

        adjacent_edge_ids.clear();
        triangle_edge_ids.clear();
    }
}


steiner_graph
steiner_graph::make_graph(std::vector<steiner_graph::node_info_type> &&__triangulation_nodes,
                          steiner_graph::base_topology_type &&__triangulation_edges,
                          std::vector<std::array<triangle_node_id_type, 3>> &&faces, float __epsilon) {

    std::vector<steiner_graph::node_info_type> triangulation_nodes(std::move(__triangulation_nodes));
    adjacency_list<steiner_graph::triangle_node_id_type, steiner_graph::triangle_edge_info_type> triangulation(
            std::move(__triangulation_edges));

    auto poly = polyhedron<steiner_graph::base_topology_type, 3>::make_polyhedron(triangulation, faces);

    std::vector<std::float16_t> r_values;
    make_node_radii(triangulation_nodes, triangulation, poly, __epsilon, r_values);

    auto table = subdivision_table::precompute(__epsilon, 0.01);
    auto subdivision_info = subdivision_table::make_subdivision_info(triangulation, triangulation_nodes, poly,
                                                                     table,
                                                                     r_values, __epsilon);
    subdivision_table s_table{std::move(table), std::move(subdivision_info)};
    r_values.clear();

    return steiner_graph(std::move(triangulation_nodes),
                         std::move(triangulation),
                         std::move(poly),
                         std::move(s_table),
                         __epsilon);
}

steiner_graph::steiner_graph(steiner_graph &&other) noexcept
        : _M_node_count(other._M_node_count),
          _M_edge_count(other._M_edge_count),
          _M_epsilon(other._M_epsilon),
          _M_base_nodes(std::move(other._M_base_nodes)),
          _M_base_topology(std::move(other._M_base_topology)),
          _M_table(std::move(other._M_table)),
          _M_polyhedron(std::move(other._M_polyhedron)) {}

steiner_graph::steiner_graph(std::vector<steiner_graph::node_info_type> &&__triangulation_nodes,
                             adjacency_list<steiner_graph::triangle_node_id_type, steiner_graph::triangle_edge_info_type> &&__triangulation_edges,
                             polyhedron<base_topology_type, 3> &&__triangles,
                             subdivision_table &&__table,
                             float __epsilon)
        :
        _M_node_count(0),
        _M_edge_count(0),
        _M_epsilon(__epsilon),
        _M_base_nodes(std::move(__triangulation_nodes)),
        _M_base_topology(std::move(__triangulation_edges)),
        _M_table(std::move(__table)),
        _M_polyhedron(std::move(__triangles)) {

    // count nodes and edges by iterating over edges in steiner info
    for (auto base_node_id: _M_base_topology.node_ids()) {
        for (auto edge: _M_base_topology.outgoing_edges(base_node_id)) {
            auto edge_id = _M_base_topology.edge_id(base_node_id, edge.destination);
            auto edge_id_inv = _M_base_topology.edge_id(edge.destination, base_node_id);

            _M_node_count += _M_table.edge(edge_id).node_count;

            // count edge x other_edge
            for (auto &&other_edge: _M_polyhedron.edges(edge_id)) {
                if (is_none(other_edge)) continue;
                if (other_edge == edge_id) {
                    _M_edge_count += _M_table.edge(other_edge).node_count - 1;
                    continue;
                }
                if (other_edge == edge_id_inv) {
                    _M_edge_count += 1;
                    continue;
                }

                _M_edge_count += (size_t) _M_table.edge(edge_id).node_count * _M_table.edge(other_edge).node_count;
            }
        }
    }
}


coordinate_t steiner_graph::node_coordinates(steiner_graph::node_id_type __id) const {
    const coordinate_t c1 = _M_base_nodes[_M_base_topology.source(__id.edge)].coordinates;
    const coordinate_t c2 = _M_base_nodes[_M_base_topology.destination(__id.edge)].coordinates;
    const auto info = _M_table.edge(__id.edge);

    return _M_table.node_coordinates(__id.edge, __id.steiner_index, c1, c2);
}

std::span<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>>
steiner_graph::outgoing_edges(node_id_type __node_id) const {
    return outgoing_edges(__node_id, __node_id);
}

std::span<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>>
steiner_graph::outgoing_edges(node_id_type __node_id, node_id_type __reached_from) const {
    static std::vector<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>> edges;
    static std::vector<coordinate_t> destination_coordinates;

    edges.clear();
    destination_coordinates.clear();

    assert (!is_none(__node_id));

    // get inverse edge for current base edge
    auto inv_edge = base_polyhedron().inverse_edge(__node_id.edge);

    // get triangles that have not been visited yet
    auto triangles = base_polyhedron().edge_faces(__node_id.edge);
    char triangle_first = 0;
    char triangle_last = 2;

    // if this node is reached via a face crossing segment, only use edges in next face
    if (__reached_from.edge != __node_id.edge && __reached_from.edge != inv_edge) [[likely]] {
        auto visited_triangles = base_polyhedron().edge_faces(__reached_from.edge);

        if (is_none(triangles[0]) || triangles[0] == visited_triangles[0] || triangles[0] == visited_triangles[1])
                [[unlikely]] {
            triangle_first = 1;
        }
        if (is_none(triangles[1]) || triangles[1] == visited_triangles[0] || triangles[1] == visited_triangles[1])
                [[unlikely]] {
            triangle_last = 1;
        }
    } else {
        if (is_none(triangles[0])) [[unlikely]] {
            triangle_first = 1;
        }
        if (is_none(triangles[1])) [[unlikely]] {
            triangle_last = 1;
        }
    }

    // make list of edges (i.e. destination/cost pairs)
    coordinate_t source_coordinate = node(__node_id).coordinates;
    auto _steiner_info = steiner_info(__node_id.edge);

    // for inverse edge
    if (__node_id.steiner_index == _steiner_info.node_count - 1) [[unlikely]] {
        steiner_graph::node_id_type destination = {inv_edge, steiner_info(inv_edge).node_count - 1};
        coordinate_t destination_coordinate = node(destination).coordinates;
        edges.push_back({destination, {}});
        destination_coordinates.push_back(destination_coordinate);
    }

    // for neighboring node on own edge
    if (__node_id.steiner_index < _steiner_info.node_count - 1) [[likely]] {
        steiner_graph::node_id_type destination = {__node_id.edge, __node_id.steiner_index + 1};
        coordinate_t destination_coordinate = node(destination).coordinates;
        edges.push_back({destination, {}});
        destination_coordinates.push_back(destination_coordinate);
    }

    // for other neighboring node on own edge
    if (__node_id.steiner_index > 0) [[likely]] {
        steiner_graph::node_id_type destination = {__node_id.edge, __node_id.steiner_index - 1};
        coordinate_t destination_coordinate = node(destination).coordinates;
        edges.push_back({destination, {}});
        destination_coordinates.push_back(destination_coordinate);
    }

    // face-crossing edges
    for (char triangle_index = triangle_first; triangle_index < triangle_last; triangle_index++) [[unlikely]] {
        assert (!is_none(triangles[triangle_index]));
        auto triangle_edges = base_polyhedron().face_edges(triangles[triangle_index]);

        for (auto base_edge_id: triangle_edges) [[likely]] {
            if (base_edge_id == __node_id.edge || base_edge_id == inv_edge) [[unlikely]]
                continue;

            auto destination_steiner_info = steiner_info(base_edge_id);

            for (int i = 0; i < destination_steiner_info.node_count; ++i) [[likely]] {
                steiner_graph::node_id_type destination = {base_edge_id, i};
                coordinate_t destination_coordinate = node(destination).coordinates;
                edges.push_back({destination, {}});
                destination_coordinates.push_back(destination_coordinate);
            }
        }
    }

    // compute distances (can be vectorized)
    for (int e = 0; e < edges.size(); ++e) [[likely]] {
        edges[e].info.cost = distance(source_coordinate, destination_coordinates[e]);
    }

    return {edges.begin(), edges.end()};
}

steiner_graph::node_info_type steiner_graph::node(steiner_graph::node_id_type __id) const {
    return {node_coordinates(__id)};
}

steiner_graph::node_id_type steiner_graph::source(steiner_graph::edge_id_type __id) {
    return {__id.source.edge, __id.source.steiner_index};
}

steiner_graph::node_id_type steiner_graph::destination(steiner_graph::edge_id_type __id) {
    return {__id.destination.edge, __id.destination.steiner_index};
}

steiner_graph::edge_info_type steiner_graph::edge(steiner_graph::edge_id_type __id) const {
    edge_info_type result;
    auto src = node(source(__id));
    auto dest = node(destination(__id));
    result.cost = distance(src.coordinates, dest.coordinates);
    return result;
}

// FIXME
steiner_graph::edge_id_type
steiner_graph::edge_id(steiner_graph::node_id_type __src, steiner_graph::node_id_type __dest) const {
    assert(__src.edge >= 0 && __src.steiner_index >= 0);
    assert(__src.edge < _M_base_topology.edge_count() &&
           __src.steiner_index < _M_table.edge(__src.edge).node_count);

    if (__dest == __src)
        return none_value<edge_id_type>;

    if (__dest.edge == __src.edge || __dest.edge == _M_polyhedron.inverse_edge(__src.edge)) {
        return {__src, __dest};
    }

    // search the edge in the base graph that __dest belongs to
    for (auto adjacent_edge: _M_polyhedron.edges(__src.edge)) {
        if (adjacent_edge == __dest.edge && __dest.steiner_index < steiner_info(adjacent_edge).node_count) {
            return {__src, __dest};
        }
    }

    return none_value<edge_id_type>;
}

bool
steiner_graph::has_edge(steiner_graph::node_id_type __src, steiner_graph::node_id_type __dest) const {
    return !is_none(edge_id(__src, __dest));
}


steiner_graph::subgraph_type steiner_graph::make_subgraph(const path_type &__route) const {
    std::vector<node_id_type> nodes(__route.nodes);
    std::vector<edge_id_type> edges;
    for (size_t i = 1; i < __route.nodes.size(); i++) {
        auto src = __route.nodes[i - 1];
        auto dest = __route.nodes[i];

        assert(has_edge(src, dest));
        edges.push_back(edge_id(src, dest));
    }
    return {*this, std::move(nodes), std::move(edges)};
}

steiner_graph::distance_type steiner_graph::path_length(const path_type &__route) const {
    distance_type result = 0;

    for (size_t i = 1; i < __route.nodes.size(); i++) {
        auto src = __route.nodes[i - 1];
        auto dest = __route.nodes[i];
        result += distance(node(src).coordinates, node(dest).coordinates);
    }

    return result;
}

subdivision_table::subdivision_edge_info
steiner_graph::steiner_info(steiner_graph::triangle_edge_id_type __id) const {
    return _M_table.edge(__id);
}

steiner_graph::node_id_iterator_type steiner_graph::node_ids(steiner_graph::triangle_edge_id_type edge) const {
    return {this, {edge, 0}, {edge + 1, 0}};
}

steiner_graph::node_id_iterator_type steiner_graph::node_ids() const {
    return {this, {0, 0}, {static_cast<edge_id_t>(_M_base_topology.edge_count()), 0}};
}


steiner_graph::node_info_type steiner_graph::node(steiner_graph::triangle_node_id_type __id) const {
    return _M_base_nodes[__id];
}

steiner_graph::node_id_iterator_type steiner_graph::node_id_iterator_type::operator++() {
    auto result = *this;
    _M_current_node.steiner_index++;
    if (_M_current_node.steiner_index >= _M_graph_ptr->_M_table.edge(_M_current_node.edge).node_count) {
        _M_current_node.steiner_index = 0;
        _M_current_node.edge++;
    }

    return result;
}


steiner_graph::node_id_iterator_type steiner_graph::node_id_iterator_type::operator++(int) {
    _M_current_node.steiner_index++;
    if (_M_current_node.steiner_index >= _M_graph_ptr->_M_table.edge(_M_current_node.edge).node_count) {
        _M_current_node.steiner_index = 0;
        _M_current_node.edge++;
    }

    return *this;
}
