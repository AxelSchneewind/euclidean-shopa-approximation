#pragma once

#include "../graph/adjacency_list.h"
#include "../graph/subgraph.h"
#include "polyhedron.h"
#include "steiner_graph.h"

#include "subdivision_table_impl.h"
#include "subdivision_info_impl.h"


#include "../util/set_minus.h"
#include <functional>
#include <span>
#include <unordered_map>
#include <cmath>


template<typename E, typename I>
std::size_t std::hash<steiner_node_id<E, I>>::operator()(const steiner_node_id<E, I> &s) const noexcept {
    std::size_t h1 = std::hash<E>{}(s.edge);
    std::size_t h2 = std::hash<I>{}(s.steiner_index);
    return h1 ^ (h2 << 1);
}

template<typename N>
std::size_t std::hash<steiner_edge_id<N>>::operator()(const steiner_edge_id<N> &s) const noexcept {
    std::size_t h1 = std::hash<N>{}(s.source);
    std::size_t h2 = std::hash<N>{}(s.destination);
    return h1 ^ (h2 << 1);
}


template<typename E, typename I>
std::ostream &operator<<(std::ostream &output, steiner_node_id<E, I> id) {
    return output << id.edge << ':' << id.steiner_index;
}


template<typename N>
std::ostream &operator<<(std::ostream &output, steiner_edge_id<N> id) {
    return output << '(' << id.source << ',' << id.destination << ')';
}


void make_node_radii(const std::vector<steiner_graph::node_info_type> &nodes,
                     const steiner_graph::base_topology_type &triangulation,
                     const steiner_graph::polyhedron_type &polyhedron, std::vector<double> &out) {
    assert(nodes.size() == triangulation.node_count());

    out.resize(triangulation.node_count());

    std::vector<steiner_graph::triangle_edge_id_type> adjacent_edge_ids;
    std::vector<steiner_graph::triangle_edge_id_type> triangle_edge_ids;
    for (auto&& node : triangulation.node_ids()) {
        adjacent_edge_ids.emplace_back(optional::none_value<steiner_graph::triangle_edge_id_type>);
        // get edges and triangles adjacent to node
        for (auto&& edge: triangulation.outgoing_edges(node)) {
            auto edge_id = triangulation.edge_id(node, edge.destination);
            adjacent_edge_ids.push_back(edge_id);

            for (auto &&triangle_edge: polyhedron.edges(edge_id))
                triangle_edge_ids.emplace_back(triangle_edge);
        }
        // get edges and triangles adjacent to node
        for (auto&& edge: triangulation.incoming_edges(node)) {
            auto inv_edge_id = triangulation.edge_id(edge.destination, node);
            adjacent_edge_ids.push_back(inv_edge_id);

            for (auto &&triangle_edge: polyhedron.edges(inv_edge_id))
                triangle_edge_ids.emplace_back(triangle_edge);
        }

        double dist = infinity<double>;
        if (!triangle_edge_ids.empty()) {
            std::sort(triangle_edge_ids.begin(), triangle_edge_ids.end());
            std::sort(adjacent_edge_ids.begin(), adjacent_edge_ids.end());

            // get edges that are only reachable via a triangle
            remove_duplicates_sorted(triangle_edge_ids);
            remove_duplicates_sorted(adjacent_edge_ids);
            int edge_count = set_minus_sorted<int>(triangle_edge_ids, adjacent_edge_ids, triangle_edge_ids);
            assert(edge_count > 0);

            // get minimal value from triangle edges
            auto c3 = nodes[node].coordinates;
            for (int i = 0; i < edge_count; i++) {
                auto const &edge_id = triangle_edge_ids[i];
                auto const &c1 = nodes[triangulation.source(edge_id)].coordinates;
                auto const &c2 = nodes[triangulation.destination(edge_id)].coordinates;
                if (c3 != c2 && c3 != c1 && c2 != c1) {
                    auto line_dist = line_distance(c1, c2, c3);
                    dist = std::min(dist, line_dist);
                    dist = std::min(dist, distance(c2, c3));
                    dist = std::min(dist, distance(c1, c3));
                }
                assert(is_infinity(dist) || (dist <= 1.01 * distance(c2, c3) && dist <= 1.01 * distance(c1, c3)));
            }
            dist = std::max(dist, 64.0 * std::numeric_limits<float>::min());
            assert(!is_infinity(dist));

            adjacent_edge_ids.clear();
            triangle_edge_ids.clear();
        }

        out[node] = dist;
    }
}


steiner_graph
steiner_graph::make_graph(std::vector<node_info_type> &&triangulation_nodes,
                          base_topology_type &&triangulation_edges,
                          std::vector<std::array<triangle_node_id_type, 3>> &&faces, double epsilon) {
    assert(triangulation_nodes.size() == triangulation_edges.node_count());

    base_topology_type triangulation(std::move(triangulation_edges));

    auto poly = polyhedron<base_topology_type, 3>::make_polyhedron(triangulation, std::move(faces));

    std::vector<double> r_values;
    make_node_radii(triangulation_nodes, triangulation, poly, r_values);

    const double min_inner_angle = M_PI / 180; // 1º
    const double minimal_relative_radius =
            std::sin(min_inner_angle) * (1 / (1 + std::sin(M_PI - min_inner_angle) / std::sin(min_inner_angle))) *
            epsilon;
    double min_r_value = 0.5;
    for (auto base_node: triangulation.node_ids()) {
        for (auto edge: triangulation.outgoing_edges(base_node)) {
            auto length = distance(triangulation_nodes[base_node].coordinates,
                                   triangulation_nodes[edge.destination].coordinates);

            double r_relative = (epsilon / 5) * (r_values[base_node] / length);

            if (r_relative <= minimal_relative_radius) continue;

            min_r_value = std::min(min_r_value, r_relative);
        }
    }
    assert(min_r_value <= 0.5);

    // auto table = subdivision_info_type::precompute(epsilon, min_r_value);
    auto subdivision_info = subdivision_info_type::make_subdivision_info(triangulation, triangulation_nodes, poly,
                                                                     // table,
                                                                     r_values, epsilon);
    r_values.clear();

    subdivision_info_type s_table{/*std::move(table),*/ std::move(subdivision_info)};

    return {std::move(triangulation_nodes),
            std::move(triangulation),
            std::move(poly),
            std::move(s_table),
            epsilon};
}


steiner_graph steiner_graph::make_graph(const steiner_graph &other, const subgraph<base_topology_type> &subgraph) {
    base_topology_type::builder builder;
    std::unordered_map<triangle_node_id_type, bool> contained;

    // mark nodes from subgraph as contained
    for (triangle_node_id_type id: subgraph.nodes)
        contained[id] = true;

    // make faces with new node ids
    std::vector<std::array<triangle_node_id_type, 3>> faces;
    for (size_t i = 0; i < other.base_polyhedron().face_count(); ++i) {
        auto &&face = other.base_polyhedron().face_edges(i);

        std::vector<triangle_node_id_type> nodes;
        nodes.emplace_back(other.base_graph().source(face[0]));
        nodes.emplace_back(other.base_graph().source(face[1]));
        nodes.emplace_back(other.base_graph().source(face[2]));
        nodes.emplace_back(other.base_graph().destination(face[0]));
        nodes.emplace_back(other.base_graph().destination(face[1]));
        nodes.emplace_back(other.base_graph().destination(face[2]));
        remove_duplicates(nodes);

        if (contained.contains(nodes[0]) && contained.contains(nodes[1]) && contained.contains(nodes[2])) {
            faces.emplace_back( std::array<triangle_node_id_type, 3>{nodes[0], nodes[1], nodes[2]}); // insert original ids
        }
    }

    // check for unconnected nodes
    contained.clear();
    for (auto &&triangle: faces) {
        contained[triangle[0]] = true;
        contained[triangle[1]] = true;
        contained[triangle[2]] = true;
    }

    // assign new node ids
    std::unordered_map<triangle_node_id_type, triangle_node_id_type> new_node_id;
    std::vector<triangle_node_info_type> nodes;
    size_t node_count = 0;
    for (triangle_node_id_type id: subgraph.nodes) {
        if (contained.contains(id) && contained[id]) {
            new_node_id[id] = node_count++;
            nodes.emplace_back(other.node(id));
        }
    }
    contained.clear();

    // apply new node ids
    for (auto &triangle: faces) {
        assert(new_node_id.contains(triangle[0]) && new_node_id.contains(triangle[1]) && new_node_id.contains(triangle[2]));
        triangle[0] = new_node_id[triangle[0]];
        triangle[1] = new_node_id[triangle[1]];
        triangle[2] = new_node_id[triangle[2]];
        std::sort(triangle.begin(), triangle.end());
        assert(!optional::is_none(triangle[0]) && !optional::is_none(triangle[1]) && !optional::is_none(triangle[2]));
    }
    new_node_id.clear();

    // make bidirectional adjacency list
    builder.add_edges_from_triangulation(faces);
    auto list = base_topology_type::make_bidirectional(builder.get());
    assert(list.node_count() == node_count);

#ifndef NDEBUG
    for (auto const &triangle: faces) {
        assert(list.has_edge(triangle[0], triangle[1]) || list.has_edge(triangle[1], triangle[0]));
        assert(list.has_edge(triangle[0], triangle[2]) || list.has_edge(triangle[2], triangle[0]));
        assert(list.has_edge(triangle[1], triangle[2]) || list.has_edge(triangle[2], triangle[1]));
    }
#endif


    return steiner_graph::make_graph(std::move(nodes), std::move(list), std::move(faces), other.epsilon());
}


steiner_graph::steiner_graph(steiner_graph &&other) noexcept
        : _node_count(other._node_count),
          _edge_count(other._edge_count),
          _epsilon(other._epsilon),
          _base_nodes(std::move(other._base_nodes)),
          _base_topology(std::move(other._base_topology)),
          _table(std::move(other._table)),
          _polyhedron(std::move(other._polyhedron)) {
    other._node_count = 0;
    other._edge_count = 0;
}

steiner_graph::steiner_graph(std::vector<node_info_type> &&triangulation_nodes,
                             adjacency_list<triangle_node_id_type, triangle_edge_info_type> &&triangulation_edges,
                             polyhedron<base_topology_type, 3> &&triangles,
                             subdivision_info_type &&table,
                             double epsilon)
        :
        _node_count(0),
        _edge_count(0),
        _epsilon(epsilon),
        _base_nodes(std::move(triangulation_nodes)),
        _base_topology(std::move(triangulation_edges)),
        _table(std::move(table)),
        _polyhedron(std::move(triangles)) {
    _node_count += _base_nodes.size();

    // count nodes by iterating over base edges
    std::size_t base_edges_checked = 0;
    for (auto &&base_node_id: _base_topology.node_ids()) {
        for (auto &&edge: _base_topology.outgoing_edges(base_node_id)) {
            const auto edge_id = _base_topology.edge_id(base_node_id, edge.destination);
            base_edges_checked++;

            // make sure each base node is only contained once
            assert (base_node_id < edge.destination);
            assert (!optional::is_none(edge_id));

            // count steiner points, but not base nodes here
            _node_count += _table.edge(edge_id).node_count - 2;

            // edges on this edge (counts (v,p_1), (p_i p_i+1), (p_k,w) and their inverses)
            _edge_count += 2 * (steiner_info(edge_id).node_count - 1);

            // edges between this edge and another one
            for (auto &&other_edge_id: _polyhedron.edges(edge_id)) {
                if (optional::is_none(other_edge_id) || other_edge_id == edge_id) continue;
                assert (_base_topology.source(other_edge_id) < _base_topology.destination(other_edge_id));

                // only count in one direction as other edge is handled separately
                _edge_count +=
                        1 * (steiner_info(edge_id).node_count - 2) * (steiner_info(other_edge_id).node_count - 2);
            }
        }
    }
    assert(base_edges_checked == _base_topology.edge_count());

    if constexpr (face_crossing_from_base_nodes) {
        // count all outgoing edges of base nodes
        for (auto&& node: base_graph().node_ids()) {
            for (auto &&reachable_edges = _polyhedron.node_edges(node); auto &&edge_id: reachable_edges) {
                if (optional::is_none(edge_id)) continue;
                assert (_base_topology.source(edge_id) < _base_topology.destination(edge_id));
                _edge_count += 2 * (steiner_info(edge_id).node_count - 2);
            }
        }
    }
}


steiner_graph::coordinate_type const& steiner_graph::node_coordinates_first(triangle_edge_id_type id) const {
    return _base_nodes[_base_topology.source(id)].coordinates;
}

steiner_graph::coordinate_type steiner_graph::node_coordinates_mid(triangle_edge_id_type id) const {
    const steiner_graph::coordinate_type &c2 = _base_nodes[_base_topology.destination(id)].coordinates;
    const steiner_graph::coordinate_type &c1 = _base_nodes[_base_topology.source(id)].coordinates;
    const auto relative = _table.relative_position_mid(id);

    return interpolate_linear(c1, c2, relative);
}

steiner_graph::coordinate_type const& steiner_graph::node_coordinates_last(triangle_edge_id_type id) const {
    return _base_nodes[_base_topology.destination(id)].coordinates;
}

steiner_graph::coordinate_type steiner_graph::node_coordinates(node_id_type id) const {
    const coordinate_type &c2 = _base_nodes[_base_topology.destination(id.edge)].coordinates;
    const coordinate_type &c1 = _base_nodes[_base_topology.source(id.edge)].coordinates;

    return _table.node_coordinates(id.edge, id.steiner_index, c1, c2);
}

steiner_graph::coordinate_type steiner_graph::node_coordinates_steiner(node_id_type id) const {
    const auto relative = _table.relative_position_steiner(id.edge, id.steiner_index);
    return interpolate_linear(node_coordinates_first(id.edge), node_coordinates_last(id.edge), relative);
}


steiner_graph::coordinate_type const& steiner_graph::node_coordinates(triangle_node_id_type id) const {
    return _base_nodes[id].coordinates;
}

std::span<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>>
steiner_graph::outgoing_edges(triangle_node_id_type base_node_id) const {
    static std::vector<internal_adjacency_list_edge<node_id_type, edge_info_type>> edges;
    static std::vector<coordinate_t> destination_coordinates;

    edges.clear();
    destination_coordinates.clear();

    if constexpr (face_crossing_from_base_nodes) {
        // NOTE: paper does not require these edges, but as bending only occurs at base nodes
        // these edges allow for less outgoing edges per steiner node
        auto &&reachable_edges = _polyhedron.node_edges(base_node_id);
        for (auto &&e: reachable_edges) [[likely]] {
            auto destination_steiner_info = steiner_info(e);

            for (short i = 1; i < destination_steiner_info.node_count - 1; ++i) [[likely]] {
                node_id_type destination = {e, i};
                coordinate_type const& destination_coordinate = node(destination).coordinates;
                edges.emplace_back(destination);
                destination_coordinates.push_back(destination_coordinate);
            }
        }
    }

    for (auto &&e: _base_topology.outgoing_edges(base_node_id)) [[likely]] {
        assert (e.destination > base_node_id);

        auto e_id = _base_topology.edge_id(base_node_id, e.destination);
        node_id_type destination = {e_id, 1};
        coordinate_type const& destination_coordinate = node(destination).coordinates;
        edges.emplace_back(destination);
        destination_coordinates.push_back(destination_coordinate);
    }
    for (auto &&e: _base_topology.incoming_edges(base_node_id)) [[likely]] {
        assert (base_node_id > e.destination);

        auto e_id = _base_topology.edge_id(e.destination, base_node_id);
        node_id_type destination(e_id, steiner_info(e_id).node_count - 2U);
        coordinate_type const& destination_coordinate = node(destination).coordinates;
        edges.emplace_back(destination);
        destination_coordinates.push_back(destination_coordinate);
    }

    // compute distances (can be vectorized)
    coordinate_type const source_coordinate = node(base_node_id).coordinates;
    for (size_t e = 0; e < edges.size(); ++e) [[likely]] {
        edges[e].info.cost = distance(source_coordinate, destination_coordinates[e]);
    }

    return {edges.begin(), edges.end()};
}

std::span<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>>
steiner_graph::outgoing_edges(node_id_type node_id) const {
    static std::vector<internal_adjacency_list_edge<node_id_type, edge_info_type>> edges;
    static std::vector<coordinate_type> destination_coordinates;

    edges.clear();
    destination_coordinates.clear();

    assert (!optional::is_none(node_id));

    if (is_base_node(node_id)) [[unlikely]]
        return outgoing_edges(base_node_id(node_id));

    // make list of edges (i.e. destination/cost pairs)
    coordinate_type source_coordinate = node(node_id).coordinates;
    auto &&info = steiner_info(node_id.edge);
    auto &&triangles = _polyhedron.edge_faces(node_id.edge);

    // for neighboring node on own edge
    if (node_id.steiner_index < info.node_count - 1) [[likely]] {
        node_id_type const destination(node_id.edge, node_id.steiner_index + 1);
        coordinate_type destination_coordinate = node(destination).coordinates;
        edges.emplace_back(destination);
        destination_coordinates.push_back(destination_coordinate);
    }

    // for other neighboring node on own edge
    if (node_id.steiner_index > 0) [[likely]] {
        node_id_type const destination(node_id.edge, node_id.steiner_index - 1);
        coordinate_type destination_coordinate = node(destination).coordinates;
        edges.emplace_back(destination);
        destination_coordinates.push_back(destination_coordinate);
    }

    // face-crossing edges
    for (int triangle_index = 0; triangle_index < 2; triangle_index++) [[unlikely]] {
        if (optional::is_none(triangles[triangle_index])) continue;

        auto&& triangle_edges = base_polyhedron().face_edges(triangles[triangle_index]);

        for (auto&& base_edge_id: triangle_edges) [[likely]] {
            if (base_edge_id == node_id.edge) [[unlikely]]
                continue;

            auto &&destination_steiner_info = steiner_info(base_edge_id);

            for (node_id_type::intra_edge_id_type i = 1; i < destination_steiner_info.node_count - 1; ++i) [[likely]] {
                node_id_type destination = {base_edge_id, i};
                coordinate_type destination_coordinate = node(destination).coordinates;
                assert(has_edge(destination, node_id));

                edges.emplace_back(destination);
                destination_coordinates.push_back(destination_coordinate);
            }
        }
    }

    // compute distances (can be vectorized)
    for (size_t e = 0; e < edges.size(); ++e) [[likely]] {
        edges[e].info.cost = distance(source_coordinate, destination_coordinates[e]);
    }

    return {edges.begin(), edges.end()};
}

steiner_graph::node_info_type steiner_graph::node(node_id_type id) const {
    return node_info_type{node_coordinates(id)};
}

steiner_graph::node_id_type steiner_graph::source(edge_id_type id) {
    return {id.source.edge, id.source.steiner_index};
}

steiner_graph::node_id_type steiner_graph::destination(edge_id_type id) {
    return {id.destination.edge, id.destination.steiner_index};
}

steiner_graph::edge_info_type steiner_graph::edge(edge_id_type id) const {
    edge_info_type result;
    auto const src = node(source(id));
    auto const dest = node(destination(id));
    result.cost = distance(src.coordinates, dest.coordinates);
    return result;
}

steiner_graph::edge_id_type
steiner_graph::edge_id(node_id_type src, node_id_type dest) const {
    assert(src.edge >= 0 && src.steiner_index >= 0);
    assert(static_cast<size_t>(src.edge) < _base_topology.edge_count() &&
           src.steiner_index < _table.edge(src.edge).node_count);
    // assert(has_edge(src, dest));

    return {src, dest};
}

// TODO fix
bool
steiner_graph::has_edge(node_id_type src, node_id_type dest) const {
    return true;
    assert(_base_topology.source(src.edge) < _base_topology.destination(src.edge));
    assert(_base_topology.source(dest.edge) < _base_topology.destination(dest.edge));

    if (dest == src)
        return false;

    if (dest.edge == src.edge && std::abs(dest.steiner_index - src.steiner_index) == 1)
        return true;

    if (is_base_node(src) && dest.steiner_index == 1) {
        auto base_src = base_node_id(src);
        for (auto edge: _base_topology.outgoing_edges(base_src)) {
            auto edge_id = _base_topology.edge_id(base_src, edge.destination);

            if (dest.edge == edge_id)
                return true;
        }
    }

    if (is_base_node(src)) {
        auto base_src = base_node_id(src);
        for (auto&& edge: _base_topology.incoming_edges(base_src)) {
            auto edge_id = _base_topology.edge_id(edge.destination, base_src);

            if (dest.edge == edge_id)
                return true;
        }

        if constexpr (face_crossing_from_base_nodes) {
            for (auto&& edge: _polyhedron.node_edges(base_src)) {
                if (dest.edge == edge)
                    return true;
            }
        }
    }

    if (is_base_node(dest) && src.steiner_index == 1) {
        auto base_dest = base_node_id(dest);
        for (auto&& edge: _base_topology.outgoing_edges(base_dest)) {
            auto edge_id = _base_topology.edge_id(base_dest, edge.destination);
            if (src.edge == edge_id)
                return true;
        }
    }

    if (is_base_node(dest)) {
        auto base_dest = base_node_id(dest);
        for (auto&& edge: _base_topology.incoming_edges(base_dest)) {
            auto edge_id = _base_topology.edge_id(edge.destination, base_dest);
            if (src.edge == edge_id)
                return true;
        }

        if constexpr (face_crossing_from_base_nodes) {
            for (auto&& edge: _polyhedron.node_edges(base_dest)) {
                if (src.edge == edge)
                    return true;
            }
        }
    }

    // face-crossing edges
    if constexpr (face_crossing_from_base_nodes) {
        auto triangles = _polyhedron.edge_faces(src.edge);
        for (unsigned char triangle_index = 0;
             triangle_index < polyhedron_type::FACE_COUNT_PER_EDGE; triangle_index++) [[unlikely]] {
                 if (optional::is_none(triangles[triangle_index])) continue;
                 auto &&triangle_edges = base_polyhedron().face_edges(triangles[triangle_index]);

                 for (auto const &base_edge_id: triangle_edges) [[likely]] {
                     if (base_edge_id == src.edge) [[unlikely]]
                         continue;

                     auto &&destination_steiner_info = steiner_info(base_edge_id);
                     if (base_edge_id == dest.edge && is_in_range(base_edge_id, 0, destination_steiner_info.node_count))
                         return true;
                 }
             }
    }

    return false;
}


steiner_graph::subgraph_type steiner_graph::make_subgraph(const path_type &route) const {
    std::vector<node_id_type> nodes(route.nodes);
    std::vector<edge_id_type> edges;
    for (size_t i = 1; i < route.nodes.size(); i++) {
        auto src = route.nodes[i - 1];
        auto dest = route.nodes[i];

        edges.emplace_back(src, dest);
    }
    return {std::move(nodes), std::move(edges)};
}

steiner_graph::distance_type steiner_graph::path_length(const path_type &route) const {
    distance_type result = 0;

    for (size_t i = 1; i < route.nodes.size(); i++) {
        auto src = route.nodes[i - 1];
        auto dest = route.nodes[i];
        result += distance(node(src).coordinates, node(dest).coordinates);
    }

    return result;
}

steiner_graph::subdivision_info_type::subdivision_edge_info const &
steiner_graph::steiner_info(triangle_edge_id_type id) const {
    return _table.edge(id);
}

steiner_graph::subdivision_info_type::subdivision_edge_info &
steiner_graph::steiner_info(triangle_edge_id_type id) {
    return _table.edge(id);
}

steiner_graph::node_id_iterator_type steiner_graph::node_ids(triangle_edge_id_type edge) const {
    return {this, {edge, 0}, {edge + 1, 0}};
}

steiner_graph::node_id_iterator_type steiner_graph::node_ids() const {
    return {this, {0, 0}, {static_cast<edge_id_t>(_base_topology.edge_count()), 0}};
}


steiner_graph::node_info_type steiner_graph::node(triangle_node_id_type id) const {
    return _base_nodes[id];
}

bool steiner_graph::is_base_node(node_id_type id) const {
    return id.steiner_index == 0 || id.steiner_index == steiner_info(id.edge).node_count - 1;
}

bool steiner_graph::is_boundary_node(triangle_node_id_type id) const {
     return _polyhedron.is_boundary_node(id);
}

bool steiner_graph::is_boundary_edge(triangle_edge_id_type id) const {
    return _polyhedron.is_boundary_edge(id);
}

steiner_graph::triangle_node_id_type steiner_graph::base_node_id(node_id_type id) const {
    assert(is_base_node(id));
    return id.steiner_index <= steiner_info(id.edge).mid_index ? base_graph().source(id.edge)
                                                                   : base_graph().destination(id.edge);
}

steiner_graph::node_id_type steiner_graph::from_base_node_id(int node) const {
    if (optional::is_none(node)) [[unlikely]]
        return optional::none_value<node_id_type>;

    for (auto edge: _base_topology.outgoing_edges(node)) [[likely]] {
        auto e_id = _base_topology.edge_id(node, edge.destination);
        return {e_id, 0};
    }

    for (auto edge: _base_topology.incoming_edges(node)) [[likely]] {
        auto e_id = _base_topology.edge_id(edge.destination, node);
        return node_id_type(e_id, steiner_info(e_id).node_count - 1U);
    }

    [[unlikely]]
    return optional::none_value<node_id_type>;
}

steiner_graph::distance_type
steiner_graph::on_edge_distance(triangle_edge_id_type edge, intra_edge_id_type first,
                                intra_edge_id_type second) const {
    distance_type relative1 = _table.relative_position(edge, first);
    distance_type relative2 = _table.relative_position(edge, second);
    distance_type length = (node_coordinates_last(edge) - node_coordinates_first(edge)).length();
    return std::abs(relative2 - relative1) * length;
}

steiner_graph::node_id_iterator_type steiner_graph::node_id_iterator_type::operator++() {
    auto result = *this;
    _current_node.steiner_index++;
    if (_current_node.steiner_index >= _graph_ptr->_table.edge(_current_node.edge).node_count) {
        _current_node.steiner_index = 0;

        _current_node.edge++;
        while (_current_node.edge < _last_node.edge &&
               _graph_ptr->base_graph().source(_current_node.edge) >=
               _graph_ptr->base_graph().destination(_current_node.edge))
            _current_node.edge++;
    }

    return result;
}


steiner_graph::node_id_iterator_type steiner_graph::node_id_iterator_type::operator++(int) {
    _current_node.steiner_index++;
    if (_current_node.steiner_index >= _graph_ptr->_table.edge(_current_node.edge).node_count) {
        _current_node.steiner_index = 0;

        _current_node.edge++;
        while (_current_node.edge < _last_node.edge &&
               _graph_ptr->base_graph().source(_current_node.edge) >=
               _graph_ptr->base_graph().destination(_current_node.edge))
            _current_node.edge++;
    }

    return *this;
}
