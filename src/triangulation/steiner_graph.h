#pragma once

#include "../graph/adjacency_list.h"
#include "../graph/base_types.h"

#include <algorithm>
#include <cassert>
#include <span>
#include <vector>

struct subdivision_edge_info {
    // relative position of the points with highest distance to other edges
    float mid_position;
    // max distance of a point on this edge to other edges
    float mid_dist;

    subdivision_edge_info invert() const {
        return subdivision_edge_info{mid_position, mid_dist};
    }
};

struct steiner_node_id {
    edge_id_t edge;
    float steiner_index;
};

struct steiner_edge_id {
    edge_id_t source_edge, destination_edge;
    float source_steiner_index, destination_steiner_index;
};

// class that stores edges (topology) of a m_adj_list
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

    steiner_graph(std::vector<steiner_graph::node_info_type> __triangulation_nodes,
                  adjacency_list<steiner_graph::triangle_edge_info_t> __triangulation_edges,
                  adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>> __triangles,
                  adjacency_list<subdivision_edge_info> __steiner_info);

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


    node_info_type node(const node_id_type &__id) const;
    edge_info_type edge(const edge_id_type &__id) const;


    std::vector<internal_adjacency_list_edge<edge_info_type>> outgoing_edges(const node_id_type &__node_id) const;;

    static adjacency_list<subdivision_edge_info>
    make_steiner_info(const adjacency_list<steiner_graph::triangle_edge_info_t> &triangulation,
                      const std::vector<steiner_graph::triangle_node_info_t> &nodes,
                      const adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>> &third_point);
};

//TODO
static_assert(Topology<steiner_graph>);

adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>>
make_third_points(adjacency_list<steiner_graph::triangle_edge_info_t> triangulation) {
    // for each edge, store the id of the 2 nodes that make up the adjacent triangles
    unidirectional_adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>>::adjacency_list_builder
            third_point_builder;

    //
    for (int i = 0; i < triangulation.edge_count(); ++i) {
        auto node1 = triangulation.forward().source(i);
        auto node2 = triangulation.forward().destination(i);

        // find nodes on adjacent edges
        std::vector<steiner_graph::triangle_node_id_t> found_nodes;
        for (auto edge: triangulation.forward().outgoing_edges(node1))
            found_nodes.push_back(edge.destination);
        for (auto edge: triangulation.forward().outgoing_edges(node2))
            found_nodes.push_back(edge.destination);
        std::sort(found_nodes.begin(), found_nodes.end());

        //
        int index = 0;
        std::array<steiner_graph::triangle_node_id_t, 2> nodes;
        for (int j = 1; j < found_nodes.size(); ++j)
            if (found_nodes[j] == found_nodes[j - 1])
                nodes[index++] = found_nodes[j];

        //
        for (int j = index; j < nodes.size(); ++j)
            nodes[j] = NO_NODE_ID;

        third_point_builder.add_edge(node1, node2, nodes);

    }
    third_point_builder.add_node(triangulation.node_count());
    return adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>>::make_bidirectional(
            std::move(third_point_builder).get());
}

adjacency_list<subdivision_edge_info>
steiner_graph::make_steiner_info(const adjacency_list<steiner_graph::triangle_edge_info_t> &triangulation,
                                 const std::vector<steiner_graph::triangle_node_info_t> &nodes,
                                 const adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>> &third_point) {
    // store subdivision information here
    unidirectional_adjacency_list<subdivision_edge_info>::adjacency_list_builder builder;

    // TODO compute subdivision info
    for (int i = 0; i < triangulation.edge_count(); ++i) {
        auto node1 = triangulation.forward().source(i);
        auto node2 = triangulation.forward().destination(i);

        auto other_nodes = third_point.forward().edge(i);

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

        // distance at the mid-point
        subdivision_edge_info edge{mid_value, mid_dist};
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

    adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>> third_points = make_third_points(triangulation);
    adjacency_list<subdivision_edge_info> steiner_info = make_steiner_info(triangulation, triangulation_nodes,
                                                                           third_points);
    return steiner_graph(triangulation_nodes, triangulation, third_points, steiner_info);
}

steiner_graph::steiner_graph(std::vector<steiner_graph::node_info_type> __triangulation_nodes,
                             adjacency_list<steiner_graph::triangle_edge_info_t> __triangulation_edges,
                             adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>> __triangles,
                             adjacency_list<subdivision_edge_info> __steiner_info)
        : triangulation_nodes(__triangulation_nodes),
          triangulation(__triangulation_edges),
          third_point(__triangles),
          steiner_info(__steiner_info),
          _M_node_count(triangulation.node_count()),
          _M_edge_count(triangulation.edge_count()) {
}


std::array<steiner_graph::triangle_edge_id_t, 4>
steiner_graph::triangle_edges(const steiner_graph::triangle_edge_id_t &__edge) const {
    std::array<triangle_edge_id_t, 4> result;

    // nodes on the given edge
    triangle_node_id_t a = triangulation.forward().source(__edge);
    triangle_node_id_t b = triangulation.forward().destination(__edge);

    // the two other nodes of the triangle
    triangle_node_id_t node1 = third_point.forward().edge(__edge)[0];
    triangle_node_id_t node2 = third_point.forward().edge(__edge)[1];

    result[0] = triangulation.forward().edge_index(a, node1);
    result[1] = triangulation.forward().edge_index(b, node1);

    if (node2 != NO_NODE_ID) {
        result[2] = triangulation.forward().edge_index(a, node2);
        result[3] = triangulation.forward().edge_index(b, node2);
    } else {
        result[2] = NO_EDGE_ID;
        result[3] = NO_EDGE_ID;
    }

    return result;
}

std::vector<internal_adjacency_list_edge<steiner_graph::edge_info_type>>
steiner_graph::outgoing_edges(const node_id_type &__node_id) const {
    using temp_edge = internal_adjacency_list_edge<steiner_graph::edge_info_type>;

    std::vector<temp_edge> result;

    // iterate over edges of adjacent triangles
    for (auto edge: triangle_edges(__node_id.edge)) {
        float relative = 0;

        auto info = steiner_info.forward().edge(edge);
        while (relative < info.mid_position) {
            relative += (info.mid_dist * relative / info.mid_position);

            auto dest_id = steiner_node_id{edge, relative};
            edge_info_type info;
            info.cost = distance_euclidian(node(__node_id).coordinates, node(dest_id).coordinates);

            temp_edge steiner_edge = { dest_id, info};
            result.push_back(steiner_edge);
        }

        // add central node
        auto mid_dest_id = steiner_node_id{edge, info.mid_position};
        edge_info_type mid_info;
        mid_info.cost = distance_euclidian(node(__node_id).coordinates, node(mid_dest_id).coordinates);
        auto mid = temp_edge{ mid_dest_id, mid_info};
        result.push_back(mid);

        relative = 1;
        while (relative > info.mid_position) {
            relative -= (info.mid_dist * relative / (1 - info.mid_position));

            auto dest_id = steiner_node_id{edge, relative};
            edge_info_type info;
            info.cost = distance_euclidian(node(__node_id).coordinates, node(dest_id).coordinates);

            temp_edge steiner_edge = { dest_id, info};
            result.push_back(steiner_edge);
        }
    }

    return result;
}

steiner_graph::node_info_type steiner_graph::node(const steiner_graph::node_id_type &__id) const {
    auto const c1 = triangulation_nodes[triangulation.forward().source(__id.edge)].coordinates;
    auto const c2 = triangulation_nodes[triangulation.forward().destination(__id.edge)].coordinates;
    coordinate_t const delta = {c2.latitude - c1.latitude, c2.longitude - c2.longitude};
    coordinate_t const position
            = {c1.latitude + __id.steiner_index * delta.latitude,
               c1.longitude * __id.steiner_index * delta.longitude};
    return node_info_type{position};
}

steiner_graph::edge_info_type steiner_graph::edge(const steiner_graph::edge_id_type &__id) const {
    // TODO generate edge info
    return {};
}
