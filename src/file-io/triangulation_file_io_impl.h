#pragma once

#include "../graph/base_types.h"
#include "triangulation_file_io.h"

#include "../graph/graph.h"
#include "../graph/adjacency_list.h"
#include "../graph/geometry.h"

#include "formatters.h"
#include "fmi_file_io.h"
#include "file_io_impl.h"

#include "../util/is_in_range.h"

#include <iomanip>



template<Topology Graph, typename formatter>
Graph
triangulation_file_io::read(std::istream &input_size, std::istream &input_nodes, std::istream &input_edges) {
    using f = formatter;
    f::skip_comments(input_size);

    std::size_t node_count{f::template read<size_t>(input_size)};
    std::size_t triangle_count{f::template read<size_t>(input_size)};

    std::vector<typename Graph::node_info_type> nodes(node_count);
    std::vector<typename Graph::adjacency_list_type::builder::edge_type> edges;
    std::vector<std::array<typename Graph::node_id_type, 3>> faces(triangle_count);

    file_io::read_nodes<typename Graph::node_info_type, f>(input_nodes, nodes);
    file_io::read_triangles<typename Graph::node_id_type, f>(input_edges, faces);

    typename Graph::adjacency_list_type::builder adj_list_builder;
    adj_list_builder.add_edges_from_triangulation(faces);

    auto adj_list = Graph::adjacency_list_type::make_bidirectional_undirected(adj_list_builder.get());
    return Graph::make_graph(std::move(nodes), std::move(adj_list));
}

steiner_graph
triangulation_file_io::read_steiner(std::istream &input_size, std::istream &input_nodes, std::istream &input_triangles,
                                    double epsilon) {
    using f = stream_encoders::encode_text;
    f::skip_comments(input_size);

    std::size_t node_count{f::template read<size_t>(input_size)};
    std::size_t triangle_count{f::template read<size_t>(input_size)};

    std::vector<steiner_graph::node_info_type> nodes;
    std::vector<steiner_graph::adjacency_list_type::builder::edge_type> edges;
    std::vector<std::array<steiner_graph::triangle_node_id_type, 3>> faces;

    if (node_count >= (std::size_t)std::numeric_limits<int>::max || triangle_count >= (std::size_t) std::numeric_limits<int>::max)
        throw std::runtime_error("node or face count to high");

    nodes.resize(node_count);
    faces.resize(triangle_count);

    file_io::read_nodes<steiner_graph::node_info_type, f>(input_nodes, {nodes.begin(), nodes.end()});
    triangle_count = file_io::read_triangles<steiner_graph::base_topology_type::node_id_type, f>(input_triangles, {faces.begin(), faces.end()});
    faces.resize(triangle_count);

    steiner_graph::adjacency_list_type::builder adj_list_builder;
    adj_list_builder.add_edges_from_triangulation(faces);

    auto adj_list = steiner_graph::adjacency_list_type::make_bidirectional(adj_list_builder.get());
    return steiner_graph::make_graph(std::move(nodes), std::move(adj_list), std::move(faces), epsilon);
}

template<>
steiner_graph triangulation_file_io::read<steiner_graph>(std::istream &input) {
    return read_steiner(input, 0.5);
}


template<Topology Graph, typename format>
void triangulation_file_io::write(std::ostream &output, const Graph &graph) {
    return;
}

template<>
void triangulation_file_io::write<steiner_graph, stream_encoders::encode_text>(std::ostream &output,
                                                                               const steiner_graph &graph) {
    stream_encoders::encode_text f;

    f.write(output, graph.base_graph().node_count());
    f.write(output, '\n');
    f.write(output, graph.base_polyhedron().face_count());
    f.write(output, '\n');

    for (int i = 0; i < graph.base_graph().node_count(); ++i) {
        auto n = graph.node(i);

        f.write(output, std::setprecision(20));
        f.write(output, n.coordinates.latitude) << ' ';
        f.write(output, n.coordinates.longitude);
        f.write(output, '\n');
    }

    for (int e = 0; e < graph.base_graph().edge_count(); ++e) {
        auto triangles = graph.base_polyhedron().edge_faces(e);
        for (auto triangle: triangles) {
            if (optional::is_none(triangle)) continue;

            auto edges = graph.base_polyhedron().face_edges(triangle);

            // only insert once (if e is the edge with the smallest id)
            if (e <= edges[0] && e <= edges[1] && e <= edges[2]) {
                auto n0 = graph.base_graph().destination(edges[0]);
                auto n1 = graph.base_graph().destination(edges[1]);
                auto n2 = graph.base_graph().destination(edges[2]);

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
