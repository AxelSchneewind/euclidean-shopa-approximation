#pragma once

#include "../graph/base_types.h"

#include <cmath>
#include <vector>

template<typename Graph, typename Labels>
struct steiner_neighbors {
private:
    Graph const&graph;
    Labels const&labels;

    double const spanner_angle;
    double const spanner_angle_cos;

    double const max_angle;
    double const max_angle_cos;

    coordinate_t _source_coordinate;

    // buffer for coordinates
    std::vector<coordinate_t> destination_coordinates;


    template<typename NodeCostPair>
    void on_edge_neighbors(NodeCostPair const&node, std::vector<NodeCostPair>&out) {
        auto const&node_id = node.node;
        auto const&reached_from = node.predecessor;
        auto&&steiner_info = graph.steiner_info(node_id.edge);

        // for neighboring node on own edge
        if (node_id.steiner_index < steiner_info.node_count - 1) [[likely]]
        {
            steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index + 1);
            if (destination != reached_from) [[likely]]
            {
                coordinate_t destination_coordinate{graph.node(destination).coordinates};
                assert(graph.has_edge(node_id, destination));
                out.emplace_back(destination, node_id, node.distance);
                destination_coordinates.emplace_back(destination_coordinate);
            }
        }

        // for other neighboring node on own edge
        if (node_id.steiner_index > 0) [[likely]]
        {
            steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index - 1);
            if (destination != reached_from) [[likely]]
            {
                coordinate_t destination_coordinate{graph.node(destination).coordinates};
                assert(graph.has_edge(node_id, destination));
                out.emplace_back(destination, node_id, node.distance);
                destination_coordinates.emplace_back(destination_coordinate);
            }
        }
    }

    template<typename NodeCostPair>
    void cone(NodeCostPair const&node,
              typename Graph::base_topology_type::edge_id_type const&edge_id,
              std::vector<NodeCostPair>&out) {
        return epsilon_spanner(node, edge_id, max_angle_cos, out);
    }


    template<typename NodeCostPair>
    void epsilon_spanner(NodeCostPair const&node,
                         typename Graph::base_topology_type::edge_id_type const&edge_id, double const&max_angle_cos,
                         coordinate_t const&direction,
                         std::vector<NodeCostPair>&out) {
        auto const&node_id = node.node;

        auto&&destination_steiner_info = graph.steiner_info(edge_id);

        // binary search for node with minimal angle using the derivative over the angle depending on steiner index
        typename Graph::node_id_type::intra_edge_id_type l = 0;
        typename Graph::node_id_type::intra_edge_id_type r = destination_steiner_info.node_count - 2;
        typename Graph::node_id_type::intra_edge_id_type m = (r + l) / 2;
        double diff = 1.0;
        while (l < r && !std::isnormal(diff)) [[likely]]
        {
            steiner_graph::node_id_type const destination{edge_id, m};
            coordinate_t const destination_coordinate{graph.node(destination).coordinates};
            double cos = angle_cos(direction, destination_coordinate - _source_coordinate);

            steiner_graph::node_id_type const destination_next(edge_id, m + 1);
            coordinate_t const destination_coordinate_next = graph.node(destination_next).coordinates;
            double cos_next = angle_cos(direction, destination_coordinate_next - _source_coordinate);

            diff = cos_next - cos;

            l = (diff > 0.0) ? m + 1 : l;
            r = (diff < 0.0) ? m - 1 : r;
            m = (l + r) / 2;
        }

        coordinate_t last_direction = direction * -1;
        for (auto j = m; j >= 1; --j) [[likely]]
        {
            steiner_graph::node_id_type const destination(edge_id, j);
            coordinate_t const destination_coordinate{graph.node(destination).coordinates};
            coordinate_t const new_direction{destination_coordinate - _source_coordinate};

            if (angle_cos(direction, new_direction) < max_angle_cos)
                [[unlikely]]
                        break;
            if (angle_cos(last_direction, new_direction) > spanner_angle_cos)
                [[likely]]
                        continue;

            assert(graph.has_edge(node_id, destination));
            // assert(!graph.is_base_node(destination));
            out.emplace_back(destination, node_id, node.distance);
            destination_coordinates.emplace_back(destination_coordinate);
            last_direction = new_direction;
        }

        last_direction = direction * -1;
        for (auto j = m + 1; j < destination_steiner_info.node_count - 1; ++j) [[likely]]
        {
            steiner_graph::node_id_type const destination(edge_id, j);
            coordinate_t const destination_coordinate{graph.node(destination).coordinates};
            coordinate_t const new_direction{destination_coordinate - _source_coordinate};

            if (angle_cos(direction, new_direction) < max_angle_cos)
                [[unlikely]]
                        break;
            if (angle_cos(last_direction, new_direction) > spanner_angle_cos)
                [[likely]]
                        continue;

            assert(graph.has_edge(node_id, destination));
            // assert(!graph.is_base_node(destination));
            out.emplace_back(destination, node_id, node.distance);
            destination_coordinates.emplace_back(destination_coordinate);
            last_direction = new_direction;
        }
    }

    template<typename NodeCostPair>
    void from_base_node(NodeCostPair const&node, std::vector<NodeCostPair>&out) {
        auto const&node_id = node.node;
        auto const&reached_from = node.node;
        auto&&base_node_id = graph.base_node_id(node.node);

        for (auto&&e: graph.base_graph().outgoing_edges(base_node_id)) [[likely]]
        {
            auto e_id = graph.base_graph().edge_id(base_node_id, e.destination);
            steiner_graph::node_id_type destination{e_id, 1};
            if (destination != node.predecessor) [[likely]]
            {
                assert(graph.has_edge(node_id, destination));
                assert(!graph.is_base_node(destination));
                out.emplace_back(destination, node_id, node.distance);
                destination_coordinates.emplace_back(graph.node(destination).coordinates);
            }
        }

        for (auto&&e: graph.base_graph().incoming_edges(base_node_id)) [[likely]]
        {
            auto e_id = graph.base_graph().edge_id(e.destination, base_node_id);
            steiner_graph::node_id_type destination(e_id, graph.steiner_info(e_id).node_count - 2);
            if (destination != node.predecessor) [[likely]]
            {
                assert(graph.has_edge(node_id, destination));
                assert(!graph.is_base_node(destination));
                out.emplace_back(destination, node.node, node.distance);
                destination_coordinates.emplace_back(graph.node(destination).coordinates);
            }
        }

        if constexpr (steiner_graph::face_crossing_from_base_nodes) {
            // face-crossing edges
            auto&&triangle_edges = graph.base_polyhedron().node_edges(base_node_id);
            for (auto base_edge_id: triangle_edges) [[likely]]
            {
                epsilon_spanner(node, base_edge_id, -1.0, _source_coordinate, out);
            }
        }
    }


    template<typename NodeCostPair>
    void from_steiner_node(NodeCostPair const&node, std::vector<NodeCostPair>&out) {
        auto const&node_id = node.node;
        auto const&reached_from = node.predecessor;

        // find first predecessor on different edge
        auto other_face_crossing_predecessor = node_id;
        auto face_crossing_predecessor = reached_from;
        while (!graph.is_base_node(face_crossing_predecessor) || face_crossing_predecessor.edge == node_id.edge &&
               face_crossing_predecessor !=
               other_face_crossing_predecessor) [[likely]]
        {
            other_face_crossing_predecessor = face_crossing_predecessor;
            face_crossing_predecessor = labels.get(face_crossing_predecessor).predecessor;
        }
        coordinate_t direction = _source_coordinate - graph.node(face_crossing_predecessor).coordinates;

        on_edge_neighbors(node, out);

        // get triangles that have not been visited yet
        auto&&triangles = graph.base_polyhedron().edge_faces(node_id.edge);
        unsigned char triangle_first = 0;
        unsigned char triangle_last = 2;

        // if this node is reached via a face crossing segment, only use edges in next face
        if (!graph.is_base_node(reached_from) && reached_from.edge != node_id.edge) [[likely]]
        {
            auto&&visited_triangles = graph.base_polyhedron().edge_faces(reached_from.edge);

            if (is_none(triangles[0]) || triangles[0] == visited_triangles[0] || triangles[0] == visited_triangles[1]) [
                [unlikely]]
            {
                triangle_first = 1;
            }
            if (is_none(triangles[1]) || triangles[1] == visited_triangles[0] || triangles[1] == visited_triangles[1]) [
                [unlikely]]
            {
                triangle_last = 1;
            }
        }
        else {
            if (is_none(triangles[0])) [[unlikely]]
            {
                triangle_first = 1;
            }
            if (is_none(triangles[1])) [[unlikely]]
            {
                triangle_last = 1;
            }
        }

        // make list of edges (i.e. destination/cost pairs)
        auto&&steiner_info = graph.steiner_info(node_id.edge);

        // face-crossing edges
        for (unsigned char triangle_index = triangle_first;
             triangle_index < triangle_last; triangle_index++) [[unlikely]]
        {
            assert(!is_none(triangles[triangle_index]));
            auto&&triangle_edges = graph.base_polyhedron().face_edges(triangles[triangle_index]);

            for (auto&&base_edge_id: triangle_edges) [[likely]]
            {
                if (base_edge_id == node_id.edge)
                    [[unlikely]]
                            continue;

                epsilon_spanner(node, base_edge_id, max_angle_cos, direction, out);
                if (face_crossing_predecessor != reached_from) {
                    epsilon_spanner(node, base_edge_id, max_angle_cos, _source_coordinate -
                                    graph.node(reached_from).coordinates, out);
                }
            }
        }
    }

public:
    steiner_neighbors(Graph const&graph, Labels const&labels)
        : graph(graph)
          , labels(labels)
          , spanner_angle(std::clamp(M_PI_2 * graph.epsilon(), 0.0, M_PI_4))
          , spanner_angle_cos(std::cos(spanner_angle))
          , max_angle{std::clamp(2 * M_PI_2 * graph.epsilon(), spanner_angle, M_PI_2)}
          , max_angle_cos{std::cos(max_angle)} {
    }

    template<typename... Args>
    steiner_neighbors(Graph const&graph, Labels const&labels, Args const&...) : steiner_neighbors(graph, labels) {
    }

    steiner_neighbors(steiner_neighbors&&) noexcept = default;

    steiner_neighbors& operator=(steiner_neighbors&&) noexcept = default;

    template<typename NodeCostPair>
    void operator()(NodeCostPair const&node, std::vector<NodeCostPair>&out) {
        auto const&node_id = node.node;
        _source_coordinate = graph.node(node_id).coordinates;

        assert(!is_none(node_id));

        if (graph.is_base_node(node_id))
            [[unlikely]]
                    from_base_node(node, out);
        else
            from_steiner_node(node, out);

        // compute distances (can be vectorized)
        assert(out.size() == destination_coordinates.size());
        for (int e = 0; e < out.size(); ++e) [[likely]]
        {
            out[e].distance += distance(_source_coordinate, destination_coordinates[e]);
        }

        destination_coordinates.clear();
    }
};
