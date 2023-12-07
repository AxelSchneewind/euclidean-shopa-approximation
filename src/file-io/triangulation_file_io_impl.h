#pragma once

#include "triangulation_file_io.h"

#include "../graph/graph.h"
#include "../graph/adjacency_list.h"
#include "../graph/geometry.h"

#include "formatters.h"
#include "fmi_file_io.h"


template<typename NodeInfo, typename NodeId, typename EdgeInfo, typename formatter>
auto
triangulation_file_io::read_triangles(std::istream &input,
               std::vector<NodeInfo> const& nodes,
               std::size_t count,
               std::vector<std::array<NodeId, 3>>& faces) {
    typename unidirectional_adjacency_list<NodeId, EdgeInfo>::adjacency_list_builder builder(nodes.size());
    for (int t = 0; t < count; t++) {
        triangle tri = {formatter::template read<size_t>(input), formatter::template read<size_t>(input), formatter::template read<size_t>(input) };
        for (int i = 0; i < 3; ++i) {
            auto next = (i + 1) % 3;
            EdgeInfo edge;
            if constexpr(requires (EdgeInfo e) { e.cost; })
                edge.cost = (float) distance(nodes[tri[i]].coordinates, nodes[tri[next]].coordinates);

            builder.add_edge(tri[i], tri[next], edge);
            builder.add_edge(tri[next], tri[i], edge);
        }
        faces.push_back(tri);
    }
    return builder;
}

template<typename NodeInfo, typename NodeId, typename EdgeInfo, typename formatter>
auto
triangulation_file_io::read_triangles(std::istream &input,
               std::vector<NodeInfo> const& nodes, std::size_t count) {
    typename unidirectional_adjacency_list<NodeId, EdgeInfo>::adjacency_list_builder builder(nodes.size());
    for (int t = 0; t < count; t++) {
        triangle tri = formatter::template read<triangle>(input);
        for (int i = 0; i < 3; ++i) {
            auto next = (i + 1) % 3;

            EdgeInfo edge;
            if constexpr (requires (EdgeInfo e) {e.cost;}) {
                edge.cost = (float) distance(nodes[tri[i]].coordinates, nodes[tri[next]].coordinates);
            }
            builder.add_edge(tri[i], tri[next], edge);
            builder.add_edge(tri[next], tri[i], edge);
        }
    }
    return builder;
}



template<Topology Graph, typename formatter>
Graph
triangulation_file_io::read(std::istream &input_size, std::istream& input_nodes, std::istream& input_edges) {
    using f = formatter;
    f::skip_comments(input_size);

    size_t node_count(f::template read<node_id_t>(input_size));
    size_t triangle_count(f::template read<edge_id_t>(input_size));

    auto nodes = fmi_file_io::read_nodes<typename Graph::node_info_type, f>(input_nodes, node_count);
    auto edges = read_triangles<typename Graph::node_info_type, typename Graph::node_id_type, typename Graph::edge_info_type, f>(input_edges, nodes, triangle_count);

    auto adj_list = Graph::adjacency_list_type::make_bidirectional_undirected(edges.get());
    return Graph::make_graph(std::move(nodes), std::move(adj_list));
}

steiner_graph triangulation_file_io::read_steiner(std::istream &input_size, std::istream& input_nodes, std::istream& input_triangles, float __epsilon) {
    using f = stream_encoders::encode_text;
    f::skip_comments(input_size);

    std::size_t node_count(f::template read<size_t>(input_size));
    std::size_t triangle_count(f::template read<size_t>(input_size));

    std::vector<std::array<steiner_graph::base_topology_type::node_id_type, 3>> faces;
    auto nodes = fmi_file_io::read_nodes<steiner_graph::node_info_type, f>(input_nodes, node_count);
    auto edges = read_triangles<steiner_graph::node_info_type, steiner_graph::base_topology_type::node_id_type, steiner_graph::base_topology_type::edge_info_type, f>(input_triangles, nodes, triangle_count, faces);

    auto adj_list = steiner_graph::adjacency_list_type::make_bidirectional_undirected(edges.get());
    return steiner_graph::make_graph(std::move(nodes), std::move(adj_list), std::move(faces), __epsilon);
}

template<>
steiner_graph triangulation_file_io::read<steiner_graph>(std::istream &input) {
    return read_steiner(input, 0.5);
}


template <>
void triangulation_file_io::write<steiner_graph, stream_encoders::encode_text> (std::ostream &output, const steiner_graph& graph) {
    stream_encoders::encode_text f;

    f.write(output, graph.node_count());
    f.write(output, '\n');
    f.write(output, graph.node_count());
    f.write(output, '\n');

    for (int i = 0; i < graph.node_count(); ++i) {
        auto n = graph.node(i);

        f.write(output, n.coordinates.latitude) << ' ';
        f.write(output, n.coordinates.longitude);
        f.write(output, '\n');
    }

    for (int e = 0; e < graph.edge_count(); ++e) {
        auto triangles = graph.base_polyhedron().edge_faces(e);
        for (auto triangle : triangles) {
            if (is_none(triangle)) continue;

            auto edges = graph.base_polyhedron().face_edges(triangle);

            // only insert once (if e is the edge with the smallest id)
            if (e <= edges[0] && e <= edges[1] && e <= edges[2]) {
                auto n0 =  graph.base_graph().source(edges[0]);
                auto n1 = graph.base_graph().source(edges[1]);
                auto n2 = graph.base_graph().source(edges[2]);

                // make sure the lowest node id appears first
                if (n0 > n1)
                    std::swap(n0, n1);
                if (n0 > n2)
                    std::swap(n0, n2);
                if (n1 > n2)
                    std::swap(n1, n2);

                f.write(output, n0);
                f.write(output, ' ');
                f.write(output, n1);
                f.write(output, ' ');
                f.write(output, n2);
                f.write(output, '\n');
            }
        }
    }
}
