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
#include <utility>
#include <vector>
#include <algorithm>


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

    auto adj_list = Graph::adjacency_list_type::make_bidirectional(adj_list_builder.get());
    return Graph::make_graph(std::move(nodes), std::move(adj_list));
}

template <SteinerGraph Graph>
Graph
triangulation_file_io::read_steiner(std::istream &input_size, std::istream &input_nodes, std::istream &input_triangles,
                                    double epsilon) {
    using f = stream_encoders::encode_text;
    f::skip_comments(input_size);

    std::size_t node_count{f::template read<size_t>(input_size)};
    std::size_t triangle_count{f::template read<size_t>(input_size)};

    std::vector<typename Graph::node_info_type> nodes;
    std::vector<typename Graph::adjacency_list_type::builder::edge_type> edges;
    std::vector<std::array<typename Graph::triangle_node_id_type, 3>> faces;

    if (node_count >= static_cast<std::size_t>(std::numeric_limits<int>::max()) || triangle_count >= static_cast<std::size_t>(std::numeric_limits<int>::max()))
        throw std::runtime_error("node or face count to high");

    nodes.resize(node_count);
    faces.resize(triangle_count);

    file_io::read_nodes<typename Graph::node_info_type, f>(input_nodes, {nodes.begin(), nodes.end()});
    triangle_count = file_io::read_triangles<typename Graph::base_topology_type::node_id_type, f>(input_triangles, {faces.begin(), faces.end()});
    faces.resize(triangle_count);

    typename Graph::adjacency_list_type::builder adj_list_builder;
    adj_list_builder.add_edges_from_triangulation(faces);

    auto adj_list = Graph::adjacency_list_type::make_bidirectional(adj_list_builder.get());
    return Graph::make_graph(std::move(nodes), std::move(adj_list), std::move(faces), epsilon);
}

template<typename Graph, typename format>
void triangulation_file_io::write(std::ostream &output, const Graph &graph) {
    throw std::runtime_error("not implemented");
}

template<SteinerGraph Graph, typename format>
void triangulation_file_io::write(std::ostream &output, const Graph &graph) {
    stream_encoders::encode_text f;

    f.write(output, graph.base_graph().node_count());
    f.write(output, '\n');
    f.write(output, graph.base_polyhedron().face_count());
    f.write(output, '\n');

    std::ranges::for_each(graph.base_nodes(), [ &f, &output](auto&& node) {
        f.write(output, std::setprecision(20));
        f.write(output, node.coordinates.y) << ' ';
        f.write(output, node.coordinates.x);
        f.write(output, '\n');
    });

    for (auto&& e : graph.base_graph().edge_ids()) {
        auto triangles = graph.base_polyhedron().edge_faces(e);
        for (auto&& triangle: triangles) {
            if (optional::is_none(triangle)) continue;

            auto edges = graph.base_polyhedron().face_edges(triangle);

            // only insert once (if e is the edge with the smallest id)
            if (e <= edges[0] && e <= edges[1] && e <= edges[2]) {
                std::array<typename Graph::triangle_edge_id_type, 6> nodes {
                        graph.base_graph().destination(edges[0]),
                        graph.base_graph().destination(edges[1]),
                        graph.base_graph().destination(edges[2]),
                        graph.base_graph().source(edges[0]),
                        graph.base_graph().source(edges[1]),
                        graph.base_graph().source(edges[2]),
                };
                std::ranges::sort(nodes);

                f.write(output, nodes[0]);
                f.write(output, ' ');
                f.write(output, nodes[2]);
                f.write(output, ' ');
                f.write(output, nodes[4]);
                f.write(output, '\n');
            }
        }
    }
}
