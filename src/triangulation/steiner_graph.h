#pragma once

#include "../graph/adjacency_list.h"
#include "../graph/base_types.h"
#include "../graph/geometry.h"

#include <algorithm>
#include <cassert>
#include <span>
#include <vector>

struct subdivision_edge_info {
    // relative position of the points with highest distance to other edges
    float mid_position;
    // max distance of a point on this edge to other edges
    float mid_dist;

    // number of steiner points on this edge (counting the source and destination node)
    int node_count;
};

struct steiner_node_id {
    edge_id_t edge;
    int steiner_index;

    bool operator>=(const steiner_node_id &other) const {
        return edge >= other.edge || steiner_index >= other.steiner_index;
    }

    bool operator==(const steiner_node_id &other) const {
        return edge == other.edge && steiner_index == other.steiner_index;
    }
};

template<>
struct std::hash<steiner_node_id> {
    std::size_t operator()(const steiner_node_id &s) const noexcept {
        std::size_t h1 = std::hash<edge_id_t>{}(s.edge);
        std::size_t h2 = std::hash<int>{}(s.steiner_index);
        return h1 ^ (h2 << 1);
    }
};


struct steiner_edge_id {
    steiner_node_id source;
    steiner_node_id destination;
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

        node_id_iterator_type &operator++() {
            current.steiner_index++;
            if (current.steiner_index >= graph->steiner_info.edge(current.edge).node_count) {
                current.steiner_index = 0;
                current.edge++;
            }

            return *this;
        }

        node_id_type &operator*() { return current; }
    };

    steiner_graph(std::vector<steiner_graph::node_info_type> &&__triangulation_nodes,
                  adjacency_list<steiner_graph::triangle_edge_info_t> &&__triangulation_edges,
                  adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>> &&__triangles,
                  adjacency_list<subdivision_edge_info> &&__steiner_info);

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

    std::span<node_info_type, std::dynamic_extent> nodes() const { throw; /* TODO */};

    node_id_iterator_type node_ids() const {
        return {this, {0, 0}, {static_cast<edge_id_t>(triangulation.edge_count()),
                               0}};
    };

    node_info_type node(const node_id_type &__id) const;

    node_id_type source(const edge_id_type &__id) const;

    node_id_type destination(const edge_id_type &__id) const;

    edge_info_type edge(const edge_id_type &__id) const;

    edge_id_type edge_id(const node_id_type &src, const node_id_type &dest) const;

    bool has_edge(const node_id_type &src, const node_id_type &dest) const {
        return edge_id(src, dest).source.edge != NO_EDGE_ID; /*ToDO*/ };

    std::vector<internal_adjacency_list_edge<node_id_type, edge_info_type>>
    outgoing_edges(const node_id_type &__node_id) const;

    static adjacency_list<subdivision_edge_info>
    make_steiner_info(const adjacency_list<steiner_graph::triangle_edge_info_t> &triangulation,
                      const std::vector<steiner_graph::triangle_node_info_t> &nodes,
                      const adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>> &third_point,
                      float __epsilon);
};

//TODO
static_assert(Topology<steiner_graph>);
static_assert(RoutableGraph<steiner_graph>);

// can be generalized to any type of polyhedron (using variable instead of static sized arrays)
adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>>
make_third_points(const adjacency_list<steiner_graph::triangle_edge_info_t> &triangulation) {
    // for each edge, store the id of the 2 nodes that make up the adjacent triangles
    unidirectional_adjacency_list<steiner_graph::triangle_node_id_t, std::array<steiner_graph::triangle_node_id_t, 2>>::adjacency_list_builder
            third_point_builder(triangulation.node_count());

    // iterate over triangulation edges
    for (auto node1: triangulation.node_ids()) {
        for (auto t_edge: triangulation.outgoing_edges(node1)) {
            auto node2 = t_edge.destination;

            // find nodes on adjacent edges
            std::vector<steiner_graph::triangle_node_id_t> found_nodes;
            for (auto edge: triangulation.outgoing_edges(node1))
                if (found_nodes.size() < 20)
                    found_nodes.push_back(edge.destination);
            for (auto edge: triangulation.outgoing_edges(node2))
                if (found_nodes.size() < 20)
                    found_nodes.push_back(edge.destination);
            std::sort(found_nodes.begin(), found_nodes.end());

            // get duplicates from found nodes
            int index = 0;
            std::array<steiner_graph::triangle_node_id_t, 2> nodes;
            for (int j = 1; j < found_nodes.size(); ++j)
                if (found_nodes[j] == found_nodes[j - 1] && (index == 0 || found_nodes[j] != nodes[index - 1]))
                    nodes[index++] = found_nodes[j];

            // fill nodes
            for (int j = index; j < nodes.size(); ++j)
                nodes[j] = NO_NODE_ID;

            third_point_builder.add_edge(node1, node2, nodes);
        }
    }

    return adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>>::make_bidirectional_undirected(
            third_point_builder.get());
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

    adjacency_list<std::array<steiner_graph::triangle_node_id_t, 2>> third_points(make_third_points(triangulation));
    adjacency_list<subdivision_edge_info> steiner_info = make_steiner_info(triangulation, triangulation_nodes,
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
