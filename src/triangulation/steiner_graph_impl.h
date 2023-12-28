#pragma once

#include "steiner_graph.h"

#include "subdivision_table_impl.h"
#include "polyhedron_impl.h"


#include "../util/set_minus.h"


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
    out.clear();
    out.resize(nodes.size());

    std::vector<steiner_graph::triangle_edge_id_type> adjacent_edge_ids;
    std::vector<steiner_graph::triangle_edge_id_type> triangle_edge_ids;
    for (int node = 0; node < triangulation.node_count(); ++node) {
        // get edges and triangles adjacent to node
        for (auto edge: triangulation.outgoing_edges(node)) {
            auto edge_id = triangulation.edge_id(node, edge.destination);
            adjacent_edge_ids.push_back(edge_id);

            for (auto&& triangle_edge: polyhedron.edges(edge_id))
                triangle_edge_ids.emplace_back(triangle_edge);
        }
        // get edges and triangles adjacent to node
        for (auto edge: triangulation.incoming_edges(node)) {
            auto inv_edge_id = triangulation.edge_id(edge.destination, node);
            adjacent_edge_ids.push_back(inv_edge_id);

            for (auto&& triangle_edge: polyhedron.edges(inv_edge_id))
                triangle_edge_ids.emplace_back(triangle_edge);
        }

        // assert(adjacent_edge_ids.size() > 0);
        // assert(triangle_edge_ids.size() > 0);

        std::sort(triangle_edge_ids.begin(), triangle_edge_ids.end());
        std::sort(adjacent_edge_ids.begin(), adjacent_edge_ids.end());

        // get edges that are only reachable via a triangle
        remove_duplicates_sorted(triangle_edge_ids);
        remove_duplicates_sorted(adjacent_edge_ids);
        int edge_count = set_minus_sorted<int>(triangle_edge_ids, adjacent_edge_ids, triangle_edge_ids);
        // assert(edge_count > 0);

        // get minimal value from triangle edges
        double dist = infinity<double>;
        auto c3 = nodes[node].coordinates;
        for (int i = 0; i < edge_count; i++) {
            auto const& edge_id = triangle_edge_ids[i];
            auto const& c1 = nodes[triangulation.source(edge_id)].coordinates;
            auto const& c2 = nodes[triangulation.destination(edge_id)].coordinates;
            if (c3 != c2 && c3 != c1 && c2 != c1) {
                auto line_dist = line_distance(c1, c2, c3); // somehow computes garbage sometimes
                dist = std::min(dist, line_dist);
                dist = std::min(dist, distance(c2, c3));
                dist = std::min(dist, distance(c1, c3));
            } else {
                dist = 0.0;
            }
            assert(is_infinity(dist) || (dist <= 1.01 * distance(c2, c3) && dist <= 1.01 * distance(c1, c3)));
        }
        dist = std::max(dist, 64.0 * std::numeric_limits<float>::min());
        out[node] = dist;
        assert(!is_infinity(dist));

        adjacent_edge_ids.clear();
        triangle_edge_ids.clear();
    }
}


steiner_graph
steiner_graph::make_graph(std::vector<steiner_graph::node_info_type> &&triangulation_nodes,
                          steiner_graph::base_topology_type &&triangulation_edges,
                          std::vector<std::array<triangle_node_id_type, 3>> &&faces, float epsilon) {
    base_topology_type triangulation(std::move(triangulation_edges));

    auto poly = polyhedron<steiner_graph::base_topology_type, 3>::make_polyhedron(triangulation, std::move(faces));

    std::vector<double> r_values;
    make_node_radii(triangulation_nodes, triangulation, poly, r_values);

    const double min_inner_angle = M_PI / 720; // 1/4º
    const double minimal_relative_radius = std::sin(min_inner_angle) * (1/(1 + std::sin(M_PI - min_inner_angle) / std::sin(min_inner_angle))) * (epsilon / 5);
    double min_r_value = 1.0;
    for (auto base_node: triangulation.node_ids()) {
        for (auto edge: triangulation.outgoing_edges(base_node)) {
            auto length = distance(triangulation_nodes[base_node].coordinates, triangulation_nodes[edge.destination].coordinates);

            double r_relative = (epsilon / 5) * (r_values[base_node] / length);
            if (r_relative <= minimal_relative_radius) { min_r_value = minimal_relative_radius; break; }

            min_r_value = std::min(min_r_value, r_relative);
        }
    }
    assert(min_r_value <= 0.5);

    auto table = subdivision_table::precompute(epsilon, min_r_value);
    auto subdivision_info = subdivision_table::make_subdivision_info(triangulation, triangulation_nodes, poly,
                                                                     table,
                                                                     r_values, epsilon);
    r_values.clear();

    subdivision_table s_table{std::move(table), std::move(subdivision_info)};

    return {std::move(triangulation_nodes),
            std::move(triangulation),
            std::move(poly),
            std::move(s_table),
            epsilon};
}

steiner_graph::steiner_graph(steiner_graph &&other) noexcept
        : _M_node_count(other._M_node_count),
          _M_edge_count(other._M_edge_count),
          _M_epsilon(other._M_epsilon),
          _M_base_nodes(std::move(other._M_base_nodes)),
          _M_base_topology(std::move(other._M_base_topology)),
          _M_table(std::move(other._M_table)),
          _M_polyhedron(std::move(other._M_polyhedron)) {
    other._M_node_count = 0;
    other._M_edge_count = 0;
}

steiner_graph::steiner_graph(std::vector<steiner_graph::node_info_type> &&triangulation_nodes,
                             adjacency_list<steiner_graph::triangle_node_id_type, steiner_graph::triangle_edge_info_type> &&triangulation_edges,
                             polyhedron<base_topology_type, 3> &&triangles,
                             subdivision_table &&table,
                             float epsilon)
        :
        _M_node_count(0),
        _M_edge_count(0),
        _M_epsilon(epsilon),
        _M_base_nodes(std::move(triangulation_nodes)),
        _M_base_topology(std::move(triangulation_edges)),
        _M_table(std::move(table)),
        _M_polyhedron(std::move(triangles)) {
    _M_node_count += _M_base_nodes.size();

    // count nodes by iterating over base edges
    std::size_t base_edges_checked = 0;
    for (auto&& base_node_id: _M_base_topology.node_ids()) {
        for (auto&& edge: _M_base_topology.outgoing_edges(base_node_id)) {
            auto edge_id = _M_base_topology.edge_id(base_node_id, edge.destination);
            base_edges_checked++;

            // make sure each base node is only contained once
            assert (base_node_id < edge.destination);
            assert (!is_none(edge_id));

            // count steiner points, but not base nodes here
            _M_node_count += _M_table.edge(edge_id).node_count - 2;

            // edges on this edge (counts (v,p_1), (p_i p_i+1), (p_k,w) and their inverses)
            _M_edge_count += 2 * (steiner_info(edge_id).node_count - 1);

            // edges between this edge and another one
            for (auto&& other_edge_id: _M_polyhedron.edges(edge_id)) {
                if (is_none(other_edge_id) || other_edge_id == edge_id) continue;
                assert (_M_base_topology.source(other_edge_id) < _M_base_topology.destination(other_edge_id));

                // only count in one direction as other edge is handled separately
                _M_edge_count += 1 * (steiner_info(edge_id).node_count - 2) * (steiner_info(other_edge_id).node_count - 2);
            }
        }
    }
    assert(base_edges_checked == _M_base_topology.edge_count());

    // count all outgoing edges of base nodes
    for (auto node: base_graph().node_ids()) {
        auto &&reachable_edges = _M_polyhedron.node_edges(node);
        for (auto&& edge_id: reachable_edges) {
            if (is_none(edge_id)) continue;
            assert (_M_base_topology.source(edge_id) < _M_base_topology.destination(edge_id));
            _M_edge_count += 2 * (steiner_info(edge_id).node_count - 2);
        }
    }
}


coordinate_t steiner_graph::node_coordinates(steiner_graph::node_id_type id) const {
    const coordinate_t& c1 = _M_base_nodes[_M_base_topology.source(id.edge)].coordinates;
    const coordinate_t& c2 = _M_base_nodes[_M_base_topology.destination(id.edge)].coordinates;

    return _M_table.node_coordinates(id.edge, id.steiner_index, c1, c2);
}

std::span<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>>
steiner_graph::outgoing_edges(node_id_type node_id) const {
    if (is_base_node(node_id)) [[unlikely]]
        return outgoing_edges(base_node_id(node_id), node_id);
    return outgoing_edges(node_id, node_id, M_PI);
}

std::span<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>>
steiner_graph::outgoing_edges(steiner_graph::triangle_node_id_type base_node_id) const {
    return outgoing_edges(base_node_id, from_base_node_id(base_node_id));
}


std::span<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>>
steiner_graph::outgoing_edges(steiner_graph::triangle_node_id_type base_node_id,
                              steiner_graph::node_id_type /*reached_from*/) const {
    static std::vector<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>> edges;
    static std::vector<coordinate_t> destination_coordinates;

    edges.clear();
    destination_coordinates.clear();

    // NOTE: paper does not require these edges, but as bending only occurs at base nodes
    // these edges allow for less outgoing edges per steiner node
    auto &&reachable_edges = _M_polyhedron.node_edges(base_node_id);
    for (auto &&e: reachable_edges) [[likely]] {
        auto destination_steiner_info = steiner_info(e);

        for (short i = 1; i < destination_steiner_info.node_count - 1; ++i) [[likely]] {
            steiner_graph::node_id_type destination = {e, i};
            coordinate_t destination_coordinate = node(destination).coordinates;
            edges.push_back({destination, {}});
            destination_coordinates.push_back(destination_coordinate);
        }
    }

    for (auto &&e: _M_base_topology.outgoing_edges(base_node_id)) [[likely]] {
        assert (e.destination > base_node_id);

        auto e_id = _M_base_topology.edge_id(base_node_id, e.destination);
        steiner_graph::node_id_type destination = {e_id, 1};
        coordinate_t destination_coordinate = node(destination).coordinates;
        edges.push_back({destination, {}});
        destination_coordinates.push_back(destination_coordinate);
    }
    for (auto &&e: _M_base_topology.incoming_edges(base_node_id)) [[likely]] {
        assert (base_node_id > e.destination);

        auto e_id = _M_base_topology.edge_id(e.destination, base_node_id);
        steiner_graph::node_id_type destination (e_id, steiner_info(e_id).node_count - 2U);
        coordinate_t destination_coordinate = node(destination).coordinates;
        edges.push_back({destination, {}});
        destination_coordinates.push_back(destination_coordinate);
    }

    // compute distances (can be vectorized)
    coordinate_t const source_coordinate = node(base_node_id).coordinates;
    for (size_t e = 0; e < edges.size(); ++e) [[likely]] {
        edges[e].info.cost = distance(source_coordinate, destination_coordinates[e]);
    }

    return {edges.begin(), edges.end()};
}

std::span<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>>
steiner_graph::outgoing_edges(node_id_type node_id, node_id_type reached_from, float /*max_angle*/) const {
    static std::vector<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>> edges;
    static std::vector<coordinate_t> destination_coordinates;

    edges.clear();
    destination_coordinates.clear();

    assert (!is_none(node_id));

    if (is_base_node(node_id)) [[unlikely]]
        return outgoing_edges(base_node_id(node_id), reached_from);

    // get triangles that have not been visited yet
    auto &&triangles = base_polyhedron().edge_faces(node_id.edge);
    unsigned char triangle_first = 0;
    unsigned char triangle_last = 2;

    // if this node is reached via a face crossing segment, only use edges in next face
    if (reached_from.edge != node_id.edge) [[likely]] {
        auto visited_triangles = base_polyhedron().edge_faces(reached_from.edge);

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
    coordinate_t source_coordinate = node(node_id).coordinates;
    coordinate_t from_coordinate = node(reached_from).coordinates;
    const auto &&_steiner_info = steiner_info(node_id.edge);

    // for neighboring node on own edge
    if (node_id.steiner_index < _steiner_info.node_count - 1) [[likely]] {
        steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index + 1);
        if (destination != reached_from) [[likely]] {
            coordinate_t destination_coordinate = node(destination).coordinates;
            edges.push_back({destination, {}});
            destination_coordinates.push_back(destination_coordinate);
        }
    }

    // for other neighboring node on own edge
    if (node_id.steiner_index > 0) [[likely]] {
        steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index - 1);
        if (destination != reached_from) [[likely]] {
            coordinate_t destination_coordinate = node(destination).coordinates;
            edges.push_back({destination, {}});
            destination_coordinates.push_back(destination_coordinate);
        }
    }

    // face-crossing edges
    for (unsigned char triangle_index = triangle_first; triangle_index < triangle_last; triangle_index++) [[unlikely]] {
        assert (!is_none(triangles[triangle_index]));
        auto triangle_edges = base_polyhedron().face_edges(triangles[triangle_index]);

        for (auto base_edge_id: triangle_edges) [[likely]] {
            if (base_edge_id == node_id.edge) [[unlikely]]
                continue;

            const auto &&destination_steiner_info = steiner_info(base_edge_id);

            for (short i = 0; i < destination_steiner_info.node_count - 1; ++i) [[likely]] {
                steiner_graph::node_id_type destination = {base_edge_id, i};
                coordinate_t destination_coordinate = node(destination).coordinates;

                // if (reached_from != node_id && angle(from_coordinate, source_coordinate, source_coordinate, destination_coordinate) >
                //     max_angle) [[likely]]
                //     continue;

                edges.push_back({destination, {}});
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

steiner_graph::node_info_type steiner_graph::node(steiner_graph::node_id_type id) const {
    return {node_coordinates(id)};
}

steiner_graph::node_id_type steiner_graph::source(steiner_graph::edge_id_type id) {
    return {id.source.edge, id.source.steiner_index};
}

steiner_graph::node_id_type steiner_graph::destination(steiner_graph::edge_id_type id) {
    return {id.destination.edge, id.destination.steiner_index};
}

steiner_graph::edge_info_type steiner_graph::edge(steiner_graph::edge_id_type id) const {
    edge_info_type result;
    auto src = node(source(id));
    auto dest = node(destination(id));
    result.cost = distance(src.coordinates, dest.coordinates);
    return result;
}

steiner_graph::edge_id_type
steiner_graph::edge_id(steiner_graph::node_id_type src, steiner_graph::node_id_type dest) const {
    assert(src.edge >= 0 && src.steiner_index >= 0);
    assert(src.edge < _M_base_topology.edge_count() &&
           src.steiner_index < _M_table.edge(src.edge).node_count);

    // return has_edge(src, dest) ? edge_id_type{src, dest} : none_value<edge_id_type>;
    return {src, dest};
}

bool
steiner_graph::has_edge(steiner_graph::node_id_type src, steiner_graph::node_id_type dest) const {
    assert (_M_base_topology.source(src.edge) < _M_base_topology.destination(src.edge));
    assert(_M_base_topology.source(dest.edge) < _M_base_topology.destination(dest.edge));

    if (dest == src)
        return false;

    if (dest.edge == src.edge && std::abs(dest.steiner_index - src.steiner_index) == 1)
        return true;

    if (is_base_node(src) &&
        (dest.steiner_index == 1 || dest.steiner_index == steiner_info(dest.edge).node_count - 1)) {
        auto base_src = base_node_id(src);
        for (auto edge: _M_base_topology.outgoing_edges(base_src)) {
            auto edge_id = _M_base_topology.edge_id(base_src, edge.destination);

            if (dest.edge == edge_id && dest.steiner_index == 1)
                return true;
        }

        for (auto edge: _M_base_topology.incoming_edges(base_src)) {
            auto edge_id = _M_base_topology.edge_id(edge.destination, base_src);

            if (dest.edge == edge_id && dest.steiner_index == steiner_info(edge_id).node_count)
                return true;
        }

        for (auto edge: _M_polyhedron.node_edges(base_src)) {
            if (dest.edge == edge)
                return true;
        }
    }

    if (is_base_node(dest) &&
        (src.steiner_index == 1 || src.steiner_index == steiner_info(src.edge).node_count - 1)) {
        auto base_dest = base_node_id(dest);
        for (auto edge: _M_base_topology.outgoing_edges(base_dest)) {
            auto edge_id = _M_base_topology.edge_id(base_dest, edge.destination);

            if (src.edge == edge_id && src.steiner_index == 1)
                return true;
        }

        for (auto edge: _M_base_topology.incoming_edges(base_dest)) {
            auto edge_id = _M_base_topology.edge_id(edge.destination, base_dest);

            if (src.edge == edge_id && src.steiner_index == steiner_info(edge_id).node_count)
                return true;
        }

        for (auto edge: _M_polyhedron.node_edges(base_dest)) {
            if (src.edge == edge)
                return true;
        }
    }

    // face-crossing edges
    auto triangles = _M_polyhedron.edge_faces(src.edge);
    for (unsigned char triangle_index = 0;
         triangle_index < polyhedron_type::FACE_COUNT_PER_EDGE; triangle_index++) [[unlikely]] {
        if (is_none(triangles[triangle_index])) continue;
        auto triangle_edges = base_polyhedron().face_edges(triangles[triangle_index]);

        for (auto const &base_edge_id: triangle_edges) [[likely]] {
            if (base_edge_id == src.edge) [[unlikely]]
                continue;

            const auto &&destination_steiner_info = steiner_info(base_edge_id);
            if (base_edge_id == dest.edge && is_between(base_edge_id, 0, destination_steiner_info.node_count))
                return true;
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

subdivision_table::subdivision_edge_info
steiner_graph::steiner_info(steiner_graph::triangle_edge_id_type id) const {
    return _M_table.edge(id);
}

steiner_graph::node_id_iterator_type steiner_graph::node_ids(steiner_graph::triangle_edge_id_type edge) const {
    return {this, {edge, 0}, {edge + 1, 0}};
}

steiner_graph::node_id_iterator_type steiner_graph::node_ids() const {
    return {this, {0, 0}, {static_cast<edge_id_t>(_M_base_topology.edge_count()), 0}};
}


steiner_graph::node_info_type steiner_graph::node(steiner_graph::triangle_node_id_type id) const {
    return _M_base_nodes[id];
}

bool steiner_graph::is_base_node(steiner_graph::node_id_type id) const {
    return id.steiner_index == 0 || id.steiner_index == steiner_info(id.edge).node_count - 1;
}

steiner_graph::triangle_node_id_type steiner_graph::base_node_id(steiner_graph::node_id_type id) const {
    assert(is_base_node(id));
    return id.steiner_index < steiner_info(id.edge).node_count / 2 ? base_graph().source(id.edge) : base_graph().destination(id.edge);
}

steiner_graph::node_id_type steiner_graph::from_base_node_id(int node) const {
    if (is_none(node)) [[unlikely]]
        return none_value<node_id_type>;

    for (auto edge: _M_base_topology.outgoing_edges(node)) {
        auto e_id = _M_base_topology.edge_id(node, edge.destination);
        return {e_id, 0};
    }

    for (auto edge: _M_base_topology.incoming_edges(node)) {
        auto e_id = _M_base_topology.edge_id(edge.destination, node);
        return {e_id, steiner_info(e_id).node_count - 1U};
    }

    [[unlikely]]
    return none_value<node_id_type>;
}

steiner_graph::node_id_iterator_type steiner_graph::node_id_iterator_type::operator++() {
    auto result = *this;
    _M_current_node.steiner_index++;
    if (_M_current_node.steiner_index >= _M_graph_ptr->_M_table.edge(_M_current_node.edge).node_count) {
        _M_current_node.steiner_index = 0;

        _M_current_node.edge++;
        while (_M_current_node.edge < _M_last_node.edge &&
               _M_graph_ptr->base_graph().source(_M_current_node.edge) >=
               _M_graph_ptr->base_graph().destination(_M_current_node.edge))
            _M_current_node.edge++;
    }

    return result;
}


steiner_graph::node_id_iterator_type steiner_graph::node_id_iterator_type::operator++(int) {
    _M_current_node.steiner_index++;
    if (_M_current_node.steiner_index >= _M_graph_ptr->_M_table.edge(_M_current_node.edge).node_count) {
        _M_current_node.steiner_index = 0;

        _M_current_node.edge++;
        while (_M_current_node.edge < _M_last_node.edge &&
               _M_graph_ptr->base_graph().source(_M_current_node.edge) >=
               _M_graph_ptr->base_graph().destination(_M_current_node.edge))
            _M_current_node.edge++;
    }

    return *this;
}
