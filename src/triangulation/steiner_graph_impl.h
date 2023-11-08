#pragma once

#include "steiner_graph.h"
#include "polyhedron_impl.h"

adjacency_list<steiner_graph::triangle_node_id_type, steiner_graph::subdivision_edge_info>
steiner_graph::make_steiner_info(
        const adjacency_list<steiner_graph::triangle_node_id_type, steiner_graph::triangle_edge_info_type> &triangulation,
        const std::vector<steiner_graph::triangle_node_info_type> &nodes,
        const polyhedron<steiner_graph::triangulation_type, 3> &__polyhedron,
        float __epsilon) {
    // store subdivision information here
    unidirectional_adjacency_list<triangle_node_id_type, subdivision_edge_info>::adjacency_list_builder builder(
            nodes.size());

    for (int i = 0; i < triangulation.edge_count(); i++) {
        auto node1 = triangulation.source(i);
        auto node2 = triangulation.destination(i);

        auto other_edges = __polyhedron.edges(i);

        // get coordinates
        auto c1 = nodes[node1].coordinates;
        auto c2 = nodes[node2].coordinates;     // TODO ignore node 2

        // get minimal angle for node1 and node2
        // treat angles > 90 degrees like 90 degrees
        float angle1 = M_PI / 2;
        float angle2 = M_PI / 2;
        for (auto edge: other_edges) {
            if (is_none(edge)) continue;

            auto node3 = triangulation.destination(edge);
            if (node3 == node1 || node3 == node2) continue;

            auto c3 = nodes[node3].coordinates;

            // compute angles
            auto a1 = angle(c1, c2, c1, c3);
            auto a2 = angle(c2, c1, c2, c3);
            if (a1 < angle1)
                angle1 = a1;
            if (a2 < angle2)
                angle2 = a2;
        }

        // relative value where the mid-point (with max distance to other edges) lies between node1 and node2
        float mid_value = 1 / (1 + std::sin(angle1) / std::sin(angle2));

        // store minimal distance from the given mid-point to any other edge, relative to this edges length
        float mid_dist = mid_value * std::sin(angle1);

        // TODO compute
        float r = 0.1F;
        int count = 8;

        // distance at the mid-point
        subdivision_edge_info edge{mid_value, mid_dist, r, count};
        builder.add_edge(node1, node2, edge);
    }

    return adjacency_list<steiner_graph::triangle_node_id_type, steiner_graph::subdivision_edge_info>::make_bidirectional(
            builder.get());
}

steiner_graph
steiner_graph::make_graph(std::vector<steiner_graph::node_info_type> &&__triangulation_nodes,
                          steiner_graph::triangulation_type &&__triangulation_edges,
                          std::vector<std::array<triangle_node_id_type, 3>> &&faces) {
    float EPSILON = 2;

    std::vector<steiner_graph::node_info_type> triangulation_nodes(std::move(__triangulation_nodes));
    adjacency_list<steiner_graph::triangle_node_id_type, steiner_graph::triangle_edge_info_type> triangulation(
            std::move(__triangulation_edges));

    auto third_points = polyhedron<steiner_graph::triangulation_type, 3>::make_polyhedron(triangulation, faces);
    auto steiner_info = make_steiner_info(triangulation, triangulation_nodes,
                                          third_points, EPSILON);

    faces.clear();

    return {std::move(triangulation_nodes), std::move(triangulation), std::move(third_points),
            std::move(steiner_info), EPSILON};
}

steiner_graph::steiner_graph(std::vector<steiner_graph::node_info_type> &&__triangulation_nodes,
                             adjacency_list<steiner_graph::triangle_node_id_type, steiner_graph::triangle_edge_info_type> &&__triangulation_edges,
                             polyhedron<triangulation_type, 3> &&__triangles,
                             adjacency_list<steiner_graph::triangle_node_id_type, subdivision_edge_info> &&__steiner_info,
                             float __epsilon)
        :
        _M_node_count(0),
        _M_edge_count(0),
        triangulation_nodes(std::move(__triangulation_nodes)),
        triangulation(std::move(__triangulation_edges)),
        _M_polyhedron(std::move(__triangles)),
        _M_steiner_info(std::move(__steiner_info)),
        _M_epsilon(__epsilon) {

    // count nodes and edges by iterating over edges in steiner info
    for (auto base_node_id: _M_steiner_info.node_ids()) {
        for (auto edge: _M_steiner_info.outgoing_edges(base_node_id)) {
            auto edge_id = _M_steiner_info.edge_id(base_node_id, edge.destination);

            _M_node_count += edge.info.node_count;

            // count edge x other_edge
            for (auto other_edge: _M_polyhedron.edges(edge_id)) {
                if (is_none(other_edge)) continue;

                _M_edge_count += edge.info.node_count * steiner_info(other_edge).node_count;
            }
        }
    }
}


std::span<const steiner_graph::triangle_edge_id_type, 10>
steiner_graph::triangle_edges(const steiner_graph::triangle_edge_id_type &__edge) const {
    return _M_polyhedron.edges(__edge);
}


// TODO make iterator for this
std::vector<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>>
steiner_graph::outgoing_edges(node_id_type __node_id) const {
    using temp_edge = internal_adjacency_list_edge<node_id_type, edge_info_type>;
    std::vector<temp_edge> result;

    auto c1 = node(__node_id).coordinates;

    // iterate over edges of adjacent triangles
    auto dest_edges = triangle_edges(__node_id.edge);
    for (auto dest_edge: dest_edges) {
        if (is_none(dest_edge)) {
            continue;
        }

        // iterate over steiner points on destination edge
        auto info = _M_steiner_info.edge(dest_edge);
        for (int steiner_index = 0; steiner_index < info.node_count; ++steiner_index) {
            steiner_node_id dest_id = {dest_edge, steiner_index};
            edge_info_type _info;

            if (__node_id == dest_id) {
                _info.cost = infinity<distance_type>();
            } else {
                _info.cost = distance(c1, node(dest_id).coordinates);
            }

            temp_edge e{dest_id, _info};
            result.push_back(e);
        }
    }

    return result;
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
    assert(__src.edge < _M_steiner_info.edge_count() && __src.steiner_index < _M_steiner_info.edge(__src.edge).node_count);

    // search the edge in the base graph, that __dest belongs to
    for (auto adjacent_edge: _M_polyhedron.edges(__src.edge)) {
        if (adjacent_edge == __dest.edge && __dest.steiner_index < steiner_info(adjacent_edge).node_count) {
            return {__src, __dest};
        }
    }

    // search the edge in the base graph, that __dest belongs to
    for (auto adjacent_edge: _M_polyhedron.edges(__dest.edge)) {
        if (adjacent_edge == __src.edge && __src.steiner_index < steiner_info(adjacent_edge).node_count) {
            return {__src, __dest};
        }
    }

    return none_value<edge_id_type>();
}

bool
steiner_graph::has_edge(steiner_graph::node_id_type __src, steiner_graph::node_id_type __dest) const {
    return !is_none(edge_id(__src, __dest));
}


steiner_graph::subgraph steiner_graph::make_subgraph(const path &__route) const {
    subgraph result;
    result.nodes = __route.nodes;

    for (size_t i = 1; i < __route.nodes.size(); i++) {
        auto src = __route.nodes[i - 1];
        auto dest = __route.nodes[i];

        assert(has_edge(src, dest));
        result.edges.push_back(edge_id(src, dest));
    }

    return result;
}

steiner_graph::distance_type steiner_graph::path_length(const path &__route) const {
    distance_type result = 0;

    for (size_t i = 1; i < __route.nodes.size(); i++) {
        auto src = __route.nodes[i - 1];
        auto dest = __route.nodes[i];
        result += distance(node(src).coordinates, node(dest).coordinates);
    }

    return result;
}

steiner_graph::subdivision_edge_info
steiner_graph::steiner_info(const steiner_graph::triangle_edge_id_type &__id) const {
    return _M_steiner_info.edge(__id);
}

steiner_graph::node_id_iterator_type steiner_graph::node_ids(steiner_graph::triangle_edge_id_type edge) const {
    return {this, {edge, 0}, {edge + 1, 0}};
}

steiner_graph::node_id_iterator_type steiner_graph::node_ids() const {
    return {this, {0, 0}, {static_cast<edge_id_t>(triangulation.edge_count()), 0}};
}


coordinate_t steiner_graph::node_coordinates(steiner_graph::node_id_type __id) const {
    const coordinate_t c1 = triangulation_nodes[triangulation.source(__id.edge)].coordinates;
    const coordinate_t c2 = triangulation_nodes[triangulation.destination(__id.edge)].coordinates;
    const auto info = steiner_info(__id.edge);

    if (__id.steiner_index == 0) {
        return c1;
    }

    if (__id.steiner_index == info.node_count - 1) {
        return interpolate_linear(c1, c2, info.mid_position);
    }

    // derivative of the distance function
    float a = info.mid_dist / info.mid_position;
    // r(v)
    float r = info.r;

    //
    float x = (float) (__id.steiner_index - 1) / (float) (info.node_count - 1);

    float relative = a * x * x * info.mid_position * info.mid_position + r;
    return interpolate_linear(c1, c2, relative);
}
