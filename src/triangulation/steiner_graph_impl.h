#pragma once
#include "steiner_graph.h"

// can be generalized to any type of polyhedron (using variable instead of static sized arrays)
adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>>
make_third_points(const adjacency_list<steiner_graph::triangle_edge_info_t> &triangulation) {
    // for each edge, store the id of the 2 nodes that make up the adjacent triangles
    unidirectional_adjacency_list<steiner_graph::triangle_node_id_t, std::array<steiner_graph::triangle_node_id_t, 2>>::adjacency_list_builder
    third_point_builder(triangulation.node_count());

    // iterate over triangulation edges
    std::vector<steiner_graph::triangle_node_id_t> found_nodes;
    for (auto node1: triangulation.node_ids()) {
        for (auto t_edge: triangulation.outgoing_edges(node1)) {
            auto node2 = t_edge.destination;

            // find nodes on adjacent edges
            for (auto edge: triangulation.outgoing_edges(node1))
                if (found_nodes.size() < 40)
                    found_nodes.push_back(edge.destination);
            for (auto edge: triangulation.outgoing_edges(node2))
                if (found_nodes.size() < 40)
                    found_nodes.push_back(edge.destination);
            std::sort(found_nodes.begin(), found_nodes.end());

            // get duplicates from found nodes
            int index = 0;
            std::array<steiner_graph::triangle_node_id_t, 2> nodes{NO_NODE_ID, NO_NODE_ID};
            for (int j = 1; j < found_nodes.size(); ++j)
                if (found_nodes[j] == found_nodes[j - 1] && (index == 0 || found_nodes[j] != nodes[index - 1]))
                    nodes[index++] = found_nodes[j];
            found_nodes.clear();

            // fill nodes
            for (int j = index; j < nodes.size(); ++j)
                nodes[j] = NO_NODE_ID;

            third_point_builder.add_edge(node1, node2, nodes);
        }
    }

    auto first = third_point_builder.get();
    auto second = adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>>::make_bidirectional_undirected(
            std::move(first));
    return second;
}

adjacency_list<subdivision_edge_info>
steiner_graph::make_steiner_info(const adjacency_list<steiner_graph::triangle_edge_info_t> &triangulation,
                                 const std::vector<steiner_graph::triangle_node_info_t> &nodes,
                                 const adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>> &third_point,
                                 float __epsilon) {
    // store subdivision information here
    unidirectional_adjacency_list<triangle_node_id_t, subdivision_edge_info>::adjacency_list_builder builder;

    // TODO compute subdivision info
    for (int i = 0; i < triangulation.edge_count(); ++i) {
        auto node1 = triangulation.source(i);
        auto node2 = triangulation.destination(i);

        auto other_nodes = third_point.edge(i);

        // get coordinates
        auto c1 = nodes[node1].coordinates;
        auto c2 = nodes[node1].coordinates;

        // store minimal angle for node1 and node2
        float angle1 = 180, angle2 = 180;
        for (auto node: other_nodes) {
            if (node == NO_NODE_ID) continue;

            auto c = nodes[node].coordinates;

            // compute angles
            auto a1 = angle(c1, c2, c1, c);
            auto a2 = angle(c2, c1, c2, c);
            if (a1 < angle1)
                angle1 = a1;
            if (a2 < angle2)
                angle2 = a2;
        }

        // relative value where the mid-point (with max distance to other edges) lies between node1 and node2
        float mid_value = 1 / (1 + std::sin(angle1) / std::sin(angle2));
        coordinate_t mid_position = c1 + (c2 - c1) * mid_value;

        // store minimal distance from the given mid-point to any other edge
        float mid_dist = 0;
        for (auto node: other_nodes) {
            if (node == NO_NODE_ID) continue;

            auto c = nodes[node].coordinates;

            // compute angles
            auto d1 = line_distance(c1, c, mid_position);
            auto d2 = line_distance(c2, c, mid_position);
            if (d1 < mid_dist)
                mid_dist = d1;
            if (d2 < mid_dist)
                mid_dist = d2;
        }


        int count = 3;

        // distance at the mid-point
        subdivision_edge_info edge{mid_value, mid_dist, count};
        builder.add_edge(node1, node2, edge);
    }

    builder.add_node(nodes.size() - 1);

    return adjacency_list<subdivision_edge_info>::make_bidirectional(std::move(builder).get());
}

steiner_graph
steiner_graph::make_graph(std::vector<steiner_graph::node_info_type> &&__triangulation_nodes,
                          steiner_graph::adjacency_list_type &&__triangulation_edges) {
    float EPSILON = 2;

    std::vector<steiner_graph::node_info_type> triangulation_nodes(std::move(__triangulation_nodes));
    adjacency_list<steiner_graph::triangle_edge_info_t> triangulation(std::move(__triangulation_edges));

    auto third_points = make_third_points(triangulation);
    auto steiner_info = make_steiner_info(triangulation, triangulation_nodes,
                                          third_points, EPSILON);
    return steiner_graph(std::move(triangulation_nodes), std::move(triangulation), std::move(third_points),
                         std::move(steiner_info));
}

steiner_graph::steiner_graph(std::vector<steiner_graph::node_info_type> &&__triangulation_nodes,
                             adjacency_list<steiner_graph::triangle_edge_info_t> &&__triangulation_edges,
                             adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>> &&__triangles,
                             adjacency_list<subdivision_edge_info> &&__steiner_info)
        :
        _M_edge_count(0), _M_node_count(0), triangulation_nodes(std::move(__triangulation_nodes)),
        triangulation(std::move(__triangulation_edges)),
        third_point(std::move(__triangles)),
        steiner_info(std::move(__steiner_info)) {

    // max nodes and edges
    node_id_type first = {0, 0};
    node_id_type end = {static_cast<edge_id_t>(triangulation.edge_count()),
                        steiner_info.edge(triangulation.edge_count()).node_count};
    node_id_iterator_type it = {this, first, end};
    for (auto id: it) {
        _M_node_count++;
        for (auto t_edge_id: outgoing_edges(id)) {
            _M_edge_count++;
        }
    }


}


std::array<steiner_graph::triangle_edge_id_t, 4>
steiner_graph::triangle_edges(const steiner_graph::triangle_edge_id_t &__edge) const {
    std::array<triangle_edge_id_t, 4> result;

    // nodes on the given edge
    triangle_node_id_t a = triangulation.source(__edge);
    triangle_node_id_t b = triangulation.destination(__edge);

    // the two other nodes of the triangle
    triangle_node_id_t node1 = third_point.edge(__edge)[0];
    triangle_node_id_t node2 = third_point.edge(__edge)[1];

    result[0] = triangulation.edge_id(a, node1);
    result[1] = triangulation.edge_id(b, node1);

    if (node2 != NO_NODE_ID) {
        result[2] = triangulation.edge_id(a, node2);
        result[3] = triangulation.edge_id(b, node2);
    } else {
        result[2] = NO_EDGE_ID;
        result[3] = NO_EDGE_ID;
    }

    return result;
}


// TODO make iterator for this
std::vector<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>>
steiner_graph::outgoing_edges(const node_id_type &__node_id) const {
    using temp_edge = internal_adjacency_list_edge<node_id_type, edge_info_type>;

    std::vector<temp_edge> result;

    // iterate over edges of adjacent triangles
    auto dest_edges = triangle_edges(__node_id.edge);
    for (auto dest_edge: dest_edges) {
        if (dest_edge == NO_EDGE_ID)
            continue;

        // iterate over steiner points on destination dest_edge
        auto info = steiner_info.edge(dest_edge);
        for (int steiner_index = 0; steiner_index < info.node_count; ++steiner_index) {
            auto dest_id = steiner_node_id{dest_edge, steiner_index};

            edge_info_type _info;
            _info.cost = distance_euclidian(node(__node_id).coordinates, node(dest_id).coordinates);


            temp_edge steiner_edge = {dest_id, _info};
            result.push_back(steiner_edge);
        }
    }

    return result;
}

steiner_graph::node_info_type steiner_graph::node(const steiner_graph::node_id_type &__id) const {
    auto const c1 = triangulation_nodes[triangulation.source(__id.edge)].coordinates;
    auto const c2 = triangulation_nodes[triangulation.destination(__id.edge)].coordinates;
    coordinate_t const delta = {c2.latitude - c1.latitude, c2.longitude - c1.longitude};

    // TODO make quadratic
    float relative = ((float) __id.steiner_index) / steiner_info.edge(__id.edge).node_count;

    coordinate_t const position
            = {c1.latitude + relative * delta.latitude,
               c1.longitude + relative * delta.longitude};
    return node_info_type{position};
}

steiner_graph::node_id_type steiner_graph::source(const steiner_graph::edge_id_type &__id) const {
    return {__id.source.edge, __id.source.steiner_index};
}

steiner_graph::node_id_type steiner_graph::destination(const steiner_graph::edge_id_type &__id) const {
    return {__id.destination.edge, __id.destination.steiner_index};
}

steiner_graph::edge_info_type steiner_graph::edge(const steiner_graph::edge_id_type &__id) const {
    edge_info_type result;
    auto src = node(source(__id));
    auto dest = node(destination(__id));
    result.cost = distance(src.coordinates, dest.coordinates);
    return result;
}

steiner_graph::edge_id_type
steiner_graph::edge_id(const steiner_graph::node_id_type &src, const steiner_graph::node_id_type &dest) const {
    return {src, dest};
}

steiner_graph::node_id_iterator_type steiner_graph::node_ids() const {
    return {this, {0, 0}, {static_cast<edge_id_t>(triangulation.edge_count()),
                           0}};
}
