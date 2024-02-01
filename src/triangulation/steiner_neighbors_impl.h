#include "steiner_neighbors.h"
#include "../graph/base_types.h"

#include <cmath>
#include <vector>

template<typename Graph, typename Labels>
template<typename NodeCostPair>
typename Graph::node_id_type steiner_neighbors<Graph, Labels>::find_face_crossing_predecessor(const NodeCostPair &node) {
    if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
        assert(!is_none(node.face_crossing_predecessor()));
        return node.face_crossing_predecessor();
    } else {
        auto const &node_id = node.node();
        auto const &reached_from = node.predecessor();

        // find first predecessor on different edge
        auto closer_face_crossing_predecessor = node_id;
        auto face_crossing_predecessor = reached_from;
        while ((face_crossing_predecessor.edge == node_id.edge ||
                _graph.is_base_node(closer_face_crossing_predecessor))
               && face_crossing_predecessor != closer_face_crossing_predecessor
                ) [[likely]] {
            closer_face_crossing_predecessor = face_crossing_predecessor;
            face_crossing_predecessor = _labels.at(face_crossing_predecessor).predecessor();
        }
        return face_crossing_predecessor;
    }
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::operator()(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    auto const &node_id = node.node();
    _source_coordinate = _graph.node(node_id).coordinates;

    assert(!is_none(node_id));

    if (_graph.is_base_node(node_id)) [[unlikely]] {
        auto base_node_id = _graph.base_node_id(node_id);
        if (_graph.is_boundary_node(base_node_id))
            [[unlikely]]
                    from_boundary_node(node, out);
        else
            from_base_node(node, out);
    } else {
        from_steiner_node(node, out);
    }

    // compute distances (can be vectorized)
    assert(out.size() == _destination_coordinates.size());
    for (int e = 0; e < out.size(); ++e) [[likely]] {
        out[e].distance() += distance(_source_coordinate, _destination_coordinates[e]);
        assert(out[e].distance() >= 0);
    }

    _destination_coordinates.clear();
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::from_base_node(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    _base_node_count++;
    auto const &node_id = node.node();
    auto const &reached_from = node.node();
    auto &&base_node_id = _graph.base_node_id(node_id);

    for (auto &&e: _graph.base_graph().outgoing_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(base_node_id, e.destination);
        steiner_graph::node_id_type destination{e_id, 1};
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            assert(!_graph.is_base_node(destination));
            out.emplace_back(destination, node_id, node.distance());
            _destination_coordinates.emplace_back(_graph.node(destination).coordinates);

            if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
                out.back().face_crossing_predecessor() = node.face_crossing_predecessor();
            }
        }
    }

    for (auto &&e: _graph.base_graph().incoming_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(e.destination, base_node_id);
        steiner_graph::node_id_type destination(e_id, _graph.steiner_info(e_id).node_count - 2);
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            assert(!_graph.is_base_node(destination));
            out.emplace_back(destination, node.node(), node.distance());
            _destination_coordinates.emplace_back(_graph.node(destination).coordinates);

            if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
                out.back().face_crossing_predecessor() = node.face_crossing_predecessor();
            }
        }
    }

    // if constexpr (steiner_graph::face_crossing_from_base_nodes) {
    //     // face-crossing edges
    //     auto &&triangle_edges = _graph.base_polyhedron().node_edges(base_node_id);
    //     for (auto base_edge_id: triangle_edges) [[likely]] {
    //         epsilon_spanner(node, base_edge_id, _max_angle_cos, _source_coordinate, out);
    //     }
    // }

    _base_node_neighbor_count += out.size();
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::from_boundary_node(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    _boundary_node_count++;
    auto const &node_id = node.node();
    auto const &reached_from = node.node();
    auto &&base_node_id = _graph.base_node_id(node_id);

    for (auto &&e: _graph.base_graph().outgoing_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(base_node_id, e.destination);
        steiner_graph::node_id_type destination{e_id, 1};
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            assert(!_graph.is_base_node(destination));
            out.emplace_back(destination, node_id, node.distance());
            _destination_coordinates.emplace_back(_graph.node(destination).coordinates);

            if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
                out.back().face_crossing_predecessor() = node.node();
            }
        }
    }

    for (auto &&e: _graph.base_graph().incoming_edges(base_node_id)) [[likely]] {
        auto e_id = _graph.base_graph().edge_id(e.destination, base_node_id);
        steiner_graph::node_id_type destination(e_id, _graph.steiner_info(e_id).node_count - 2);
        if (destination != node.predecessor()) [[likely]] {
            assert(_graph.has_edge(node_id, destination));
            assert(!_graph.is_base_node(destination));
            out.emplace_back(destination, node.node(), node.distance());
            _destination_coordinates.emplace_back(_graph.node(destination).coordinates);

            if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
                out.back().face_crossing_predecessor() = node.node();
            }
        }
    }

    if constexpr (steiner_graph::face_crossing_from_base_nodes) {
        // face-crossing edges: make epsilon spanner in all directions
        auto &&triangle_edges = _graph.base_polyhedron().node_edges(base_node_id);
        for (auto &&base_edge_id: triangle_edges) [[likely]] {
            epsilon_spanner(node, base_edge_id, -1.0, _source_coordinate, out);
        }
    }

    _boundary_node_neighbor_count += out.size();
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::from_steiner_node(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    _steiner_point_count++;
    auto const &node_id = node.node();
    auto const &reached_from = node.predecessor();

    auto face_crossing_predecessor = find_face_crossing_predecessor(node);
    coordinate_t direction = _source_coordinate - _graph.node(face_crossing_predecessor).coordinates;

    //
    on_edge_neighbors(node, out);

    // get triangles that have not been visited yet
    auto &&triangles = _graph.base_polyhedron().edge_faces(node_id.edge);
    unsigned char triangle_first, triangle_last;

    // if this node is reached via a face crossing segment, only use edges in next face
    if (!_graph.is_base_node(reached_from) && reached_from.edge != node_id.edge) [[likely]] {
        auto &&visited_triangles = _graph.base_polyhedron().edge_faces(reached_from.edge);
        triangle_first = (is_none(triangles[0]) || triangles[0] == visited_triangles[0] || triangles[0] == visited_triangles[1]);
        triangle_last = 2 - (is_none(triangles[1]) || triangles[1] == visited_triangles[0] || triangles[1] == visited_triangles[1]);
    } else {
        triangle_first = (is_none(triangles[0]));
        triangle_last = 2 - (is_none(triangles[1]));
    }

    // make list of edges (i.e. destination/cost pairs)
    auto &&steiner_info = _graph.steiner_info(node_id.edge);

    // face-crossing edges
    int num_on_edge_neighbors = out.size();
    for (unsigned char triangle_index = triangle_first;
         triangle_index < triangle_last; triangle_index++) [[unlikely]] {
        assert(!is_none(triangles[triangle_index]));
        auto &&triangle_edges = _graph.base_polyhedron().face_edges(triangles[triangle_index]);

        for (auto &&base_edge_id: triangle_edges) [[likely]] {
            if (base_edge_id == node_id.edge) [[unlikely]]
                continue;

            // only one neighbor is necessary, continue if found
            add_min_angle_neighbor(node, base_edge_id, _max_angle_cos, direction, out);

            // required in paper, but probably not on unweighted triangulations
            if (face_crossing_predecessor != reached_from) [[unlikely]] {
                add_min_angle_neighbor(node, base_edge_id, _max_angle_cos,
                                       _source_coordinate - _graph.node(reached_from).coordinates, out);
            }
        }
    }

    _steiner_point_neighbor_count += out.size();
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::epsilon_spanner(const NodeCostPair &node, const typename Graph::base_topology_type::edge_id_type &edge_id,
                                                       const double &max_angle_cos, const coordinate_t &direction,
                                                       std::vector<NodeCostPair> &out) {
    auto const &node_id = node.node();
    auto &&destination_steiner_info = _graph.steiner_info(edge_id);

    auto next = add_min_angle_neighbor(node, edge_id, max_angle_cos, direction, out);
    if (is_none(next)) [[unlikely]]
        return;

    coordinate_t last_direction = direction * -1;
    for (auto j = next.steiner_index - 1; j >= 1; --j) [[likely]] {
        steiner_graph::node_id_type const destination(edge_id, j);
        coordinate_t const destination_coordinate{_graph.node(destination).coordinates};
        coordinate_t const new_direction{destination_coordinate - _source_coordinate};

        if (angle_cos(last_direction, new_direction) > _spanner_angle_cos) [[likely]]
            continue;
        if (angle_cos(direction, new_direction) < max_angle_cos) [[unlikely]]
            break;

        assert(_graph.has_edge(node_id, destination));
        out.emplace_back(destination, node_id, node.distance());
        _destination_coordinates.emplace_back(destination_coordinate);
        last_direction = new_direction;

        if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
            out.back().face_crossing_predecessor() = node.node();
        }
    }

    last_direction = direction * -1;
    for (auto j = next.steiner_index + 1; j < destination_steiner_info.node_count - 1; ++j) [[likely]] {
        steiner_graph::node_id_type const destination(edge_id, j);
        coordinate_t const destination_coordinate{_graph.node(destination).coordinates};
        coordinate_t const new_direction{destination_coordinate - _source_coordinate};

        if (angle_cos(last_direction, new_direction) > _spanner_angle_cos) [[likely]]
            continue;
        if (angle_cos(direction, new_direction) < max_angle_cos) [[unlikely]]
            break;

        assert(_graph.has_edge(node_id, destination));
        out.emplace_back(destination, node_id, node.distance());
        _destination_coordinates.emplace_back(destination_coordinate);
        last_direction = new_direction;

        if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
            out.back().face_crossing_predecessor() = node.node();
        }
    }
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
inline typename Graph::node_id_type
steiner_neighbors<Graph, Labels>::add_min_angle_neighbor(const NodeCostPair &node, const typename Graph::base_topology_type::edge_id_type &edge_id,
                                                         const double &max_angle_cos, const coordinate_t &direction,
                                                         std::vector<NodeCostPair> &out) {
    auto next = find_min_angle_neighbor<NodeCostPair>(edge_id, direction);

    // add edge with minimal angle
    coordinate_t const destination_coordinate{_graph.node(next).coordinates};

    if (angle_cos(destination_coordinate - _source_coordinate, direction) >= max_angle_cos) [[unlikely]] {
        out.emplace_back(next, node.node(), node.distance());
        _destination_coordinates.emplace_back(destination_coordinate);

        if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
            out.back().face_crossing_predecessor() = node.node();
        }

        return next;
    } else {
        return none_value<typename Graph::node_id_type>;
    }
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
typename Graph::node_id_type
steiner_neighbors<Graph, Labels>::find_min_angle_neighbor(const typename Graph::base_topology_type::edge_id_type &edge_id,
                                                          const coordinate_t &direction) {
    auto &&destination_steiner_info = _graph.steiner_info(edge_id);

    // binary search for node with minimal angle using the derivative over the angle depending on steiner index
    typename Graph::node_id_type::intra_edge_id_type l = 0;
    typename Graph::node_id_type::intra_edge_id_type r = destination_steiner_info.node_count - 2;
    typename Graph::node_id_type::intra_edge_id_type m = (r + l) / 2;
    double diff = 1.0;
    while (l < r && std::isnormal(diff)) [[likely]] {
        steiner_graph::node_id_type const destination_next(edge_id, m + 1);
        coordinate_t const destination_coordinate_next = {_graph.node(destination_next).coordinates - _source_coordinate};
        diff = angle_cos(direction, destination_coordinate_next);

        steiner_graph::node_id_type const destination{edge_id, m};
        coordinate_t const destination_coordinate{_graph.node(destination).coordinates - _source_coordinate};
        diff -= angle_cos(direction, destination_coordinate);

        int right = (diff > 0.0);
        int left = !right;
        l = right * (m + 1) + left * l;
        r = left * (m - 1) + right * r;
        m = (l + r) / 2;
    }
    assert(diff == 0 || std::isnormal(diff));

    return {edge_id, m};
}

template<typename Graph, typename Labels>
template<typename NodeCostPair>
void steiner_neighbors<Graph, Labels>::on_edge_neighbors(const NodeCostPair &node, std::vector<NodeCostPair> &out) {
    auto const &node_id = node.node();
    auto const &reached_from = node.predecessor();
    auto &&steiner_info = _graph.steiner_info(node_id.edge);

    // for neighboring node on own edge
    if (node_id.steiner_index < steiner_info.node_count - 1) [[likely]] {
        steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index + 1);
        if (destination != reached_from) [[likely]] {
            coordinate_t destination_coordinate{_graph.node(destination).coordinates};
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node_id, node.distance());
            _destination_coordinates.emplace_back(destination_coordinate);

            if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
                out.back().face_crossing_predecessor() = node.face_crossing_predecessor();
            }
        }
    }

    // for other neighboring node on own edge
    if (node_id.steiner_index > 0) [[likely]] {
        steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index - 1);
        if (destination != reached_from) [[likely]] {
            coordinate_t destination_coordinate{_graph.node(destination).coordinates};
            assert(_graph.has_edge(node_id, destination));
            out.emplace_back(destination, node_id, node.distance());
            _destination_coordinates.emplace_back(destination_coordinate);

            if constexpr (HasFaceCrossingPredecessor<NodeCostPair, Graph>) {
                out.back().face_crossing_predecessor() = node.face_crossing_predecessor();
            }
        }
    }
}
