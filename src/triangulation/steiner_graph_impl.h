#pragma once

#include "steiner_graph.h"

#include "subdivision_table_impl.h"
#include "polyhedron_impl.h"


#include "../util/set_minus.h"


template<typename E, typename I>
std::size_t std::hash<steiner_node_id<E, I>>::operator()(const steiner_node_id<E, I> &__s) const noexcept {
    std::size_t h1 = std::hash<E>{}(__s.edge);
    std::size_t h2 = std::hash<I>{}(__s.steiner_index);
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


void make_node_radii(
        const std::vector<steiner_graph::node_info_type> &__nodes,
        const steiner_graph::base_topology_type &__triangulation,
        const steiner_graph::polyhedron_type &__polyhedron,
        float __epsilon,
        std::vector<float> &__out) {
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

    std::vector<float> r_values;
    make_node_radii(triangulation_nodes, triangulation, poly, __epsilon, r_values);

    constexpr float minimal_relative_radius = 0.000000001;
    float min_r_value = 1.;
    for (auto base_node: triangulation.node_ids()) {
        for (auto edge: triangulation.outgoing_edges(base_node)) {
            auto edge_id = triangulation.edge_id(base_node, edge.destination);

            auto length = distance(triangulation_nodes[base_node].coordinates,
                                   triangulation_nodes[edge.destination].coordinates);

            float r_relative = (__epsilon / 5) * r_values[base_node] / length;
            if (r_relative <= minimal_relative_radius) continue;

            min_r_value = std::min(min_r_value, r_relative);
        }
    }

    auto table = subdivision_table::precompute(__epsilon, min_r_value);
    auto subdivision_info = subdivision_table::make_subdivision_info(triangulation, triangulation_nodes, poly,
                                                                     table,
                                                                     r_values, __epsilon);
    subdivision_table s_table{std::move(table), std::move(subdivision_info)};
    r_values.clear();

    return {std::move(triangulation_nodes),
            std::move(triangulation),
            std::move(poly),
            std::move(s_table),
            __epsilon};
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

    _M_node_count += _M_base_nodes.size();

    // count nodes by iterating over base edges
    for (auto base_node_id: _M_base_topology.node_ids()) {
        for (auto edge: _M_base_topology.outgoing_edges(base_node_id)) {
            if (base_node_id >= edge.destination) continue;

            auto edge_id = _M_base_topology.edge_id(base_node_id, edge.destination);
            auto edge_id_inv = _M_base_topology.edge_id(edge.destination, base_node_id);

            _M_node_count += _M_table.edge(edge_id).node_count - 2; // don't count base nodes here

            // edges on this edge
            _M_edge_count += 2 * (steiner_info(edge_id).node_count - 1);
            // edges between this edge and another one
            for (auto other_edge_id: _M_polyhedron.edges(edge_id)) {
                if (is_none(other_edge_id)) continue;
                if (other_edge_id == edge_id) continue;
                assert (_M_base_topology.source(other_edge_id) < _M_base_topology.destination(other_edge_id));
                _M_edge_count += 1 * (steiner_info(edge_id).node_count - 2) * (steiner_info(other_edge_id).node_count -
                                                                               2); // only count in one direction as other edge is handled separately
            }
        }
    }

    // count all outgoing edges of base nodes
    for (auto node: base_graph().node_ids()) {
        auto &&reachable_edges = _M_polyhedron.node_edges(node);
        for (auto edge_id: reachable_edges) {
            if (is_none(edge_id)) continue;
            assert (_M_base_topology.source(edge_id) < _M_base_topology.destination(edge_id));
            _M_edge_count += 2 * (steiner_info(edge_id).node_count - 2);
        }

        // auto&& edges = outgoing_edges(node);
        // for (auto edge: edges) {
        //     auto dest = edge.destination;
        //     assert(!is_base_node(dest));
        //     _M_edge_count += 2;
        // }
    }

    // count all edges between steiner points by iterating over base edges
    // for (auto node: node_ids()) {
    //     auto&& edges = outgoing_edges(node);
    //     for (auto edge: edges) {
    //         auto dest = edge.destination;

    //         if (is_base_node(node) || is_base_node(dest)) continue;

    //         _M_edge_count++;
    //     }
    // }


}


coordinate_t steiner_graph::node_coordinates(steiner_graph::node_id_type __id) const {
    const coordinate_t c1 = _M_base_nodes[_M_base_topology.source(__id.edge)].coordinates;
    const coordinate_t c2 = _M_base_nodes[_M_base_topology.destination(__id.edge)].coordinates;

    return _M_table.node_coordinates(__id.edge, __id.steiner_index, c1, c2);
}

std::span<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>>
steiner_graph::outgoing_edges(node_id_type __node_id) const {
    if (is_base_node(__node_id)) [[unlikely]]
        return outgoing_edges(base_node_id(__node_id), __node_id);
    return outgoing_edges(__node_id, __node_id, M_PI);
}

std::span<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>>
steiner_graph::outgoing_edges(steiner_graph::triangle_node_id_type __base_node_id) const {
    return outgoing_edges(__base_node_id, from_base_node_id(__base_node_id));
}


std::span<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>>
steiner_graph::outgoing_edges(steiner_graph::triangle_node_id_type __base_node_id,
                              steiner_graph::node_id_type __reached_from) const {

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

steiner_graph::edge_id_type
steiner_graph::edge_id(steiner_graph::node_id_type __src, steiner_graph::node_id_type __dest) const {
    assert(__src.edge >= 0 && __src.steiner_index >= 0);
    assert(__src.edge < _M_base_topology.edge_count() &&
           __src.steiner_index < _M_table.edge(__src.edge).node_count);

    return has_edge(__src, __dest) ? edge_id_type{__src, __dest} : none_value<edge_id_type>;
}

bool
steiner_graph::has_edge(steiner_graph::node_id_type __src, steiner_graph::node_id_type __dest) const {
    assert (_M_base_topology.source(__src.edge) < _M_base_topology.destination(__src.edge));
    assert(_M_base_topology.source(__dest.edge) < _M_base_topology.destination(__dest.edge));

    if (__dest == __src)
        return false;

    if (__dest.edge == __src.edge && std::abs(__dest.steiner_index - __src.steiner_index) == 1)
        return true;

    if (is_base_node(__src) &&
        (__dest.steiner_index == 1 || __dest.steiner_index == steiner_info(__dest.edge).node_count - 1)) {
        auto base_src = base_node_id(__src);
        for (auto edge: _M_base_topology.outgoing_edges(base_src)) {
            auto edge_id = _M_base_topology.edge_id(base_src, edge.destination);

            if (__dest.edge == edge_id && __dest.steiner_index == 1)
                return true;
        }

        for (auto edge: _M_base_topology.incoming_edges(base_src)) {
            auto edge_id = _M_base_topology.edge_id(edge.destination, base_src);

            if (__dest.edge == edge_id && __dest.steiner_index == steiner_info(edge_id).node_count)
                return true;
        }

        for (auto edge: _M_polyhedron.node_edges(base_src)) {
            if (__dest.edge == edge)
                return true;
        }
    }

    if (is_base_node(__dest) &&
        (__src.steiner_index == 1 || __src.steiner_index == steiner_info(__src.edge).node_count - 1)) {
        auto base_dest = base_node_id(__dest);
        for (auto edge: _M_base_topology.outgoing_edges(base_dest)) {
            auto edge_id = _M_base_topology.edge_id(base_dest, edge.destination);

            if (__src.edge == edge_id && __src.steiner_index == 1)
                return true;
        }

        for (auto edge: _M_base_topology.incoming_edges(base_dest)) {
            auto edge_id = _M_base_topology.edge_id(edge.destination, base_dest);

            if (__src.edge == edge_id && __src.steiner_index == steiner_info(edge_id).node_count)
                return true;
        }

        for (auto edge: _M_polyhedron.node_edges(base_dest)) {
            if (__src.edge == edge)
                return true;
        }
    }

    // face-crossing edges
    auto triangles = _M_polyhedron.edge_faces(__src.edge);
    for (unsigned char triangle_index = 0;
         triangle_index < polyhedron_type::FACE_COUNT_PER_EDGE; triangle_index++) [[unlikely]] {
        if (is_none(triangles[triangle_index])) continue;
        auto triangle_edges = base_polyhedron().face_edges(triangles[triangle_index]);

        for (auto const &base_edge_id: triangle_edges) [[likely]] {
            if (base_edge_id == __src.edge) [[unlikely]]
                continue;

            const auto &&destination_steiner_info = steiner_info(base_edge_id);
            if (base_edge_id == __dest.edge && is_between(base_edge_id, 0, destination_steiner_info.node_count))
                return true;
        }
    }

    return false;
}


steiner_graph::subgraph_type steiner_graph::make_subgraph(const path_type &__route) const {
    std::vector<node_id_type> nodes(__route.nodes);
    std::vector<edge_id_type> edges;
    for (size_t i = 1; i < __route.nodes.size(); i++) {
        auto src = __route.nodes[i - 1];
        auto dest = __route.nodes[i];

        edges.emplace_back(src, dest);
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

bool steiner_graph::is_base_node(steiner_graph::node_id_type __id) const {
    return __id.steiner_index == 0 || __id.steiner_index == steiner_info(__id.edge).node_count - 1;
}

steiner_graph::triangle_node_id_type steiner_graph::base_node_id(steiner_graph::node_id_type __id) const {
    assert(is_base_node(__id));
    return __id.steiner_index == 0 ? base_graph().source(__id.edge) : base_graph().destination(__id.edge);
}

steiner_graph::node_id_type steiner_graph::from_base_node_id(int __node) const {
    if (is_none(__node)) [[unlikely]]
        return none_value<node_id_type>;

    for (auto edge: _M_base_topology.outgoing_edges(__node)) {
        auto e_id = _M_base_topology.edge_id(__node, edge.destination);
        return {e_id, 0};
    }

    for (auto edge: _M_base_topology.incoming_edges(__node)) {
        auto e_id = _M_base_topology.edge_id(edge.destination, __node);
        return {e_id, steiner_info(e_id).node_count - 1U};
    }

    [[unlikely]]
            throw std::runtime_error("node has no incident edges");
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
