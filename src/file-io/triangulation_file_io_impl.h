#pragma once

#include "triangulation_file_io.h"

#include "../graph/graph_impl.h"
#include "../graph/adjacency_list_impl.h"
#include "../graph/geometry.h"

#include "formatters_impl.h"

template<Topology Graph, typename formatter>
Graph
triangulation_file_io::read(std::istream &input) {
    using f = formatter;
    f::skip_comments(input);

    size_t node_count(f::template read<node_id_t>(input));
    size_t triangle_count(f::template read<edge_id_t>(input));

    // read nodes
    std::vector<typename Graph::node_info_type> nodes;
    for (int i = 0; i < node_count; ++i) {
        node_t n;
        n.coordinates = f::template read<coordinate_t>(input);
        nodes.push_back(n);
    }

    // build adjacency list
    typename unidirectional_adjacency_list<node_id_t, edge_t>::adjacency_list_builder builder(node_count);

    // read triangles and generate edges from them
    for (int t = 0; t < triangle_count; t++) {
        triangle tri = f::template read<triangle>(input);
        for (int i = 0; i < 3; ++i) {
            auto next = (i + 1) % 3;
            edge_t edge;
            edge.cost = (float) distance(nodes[tri[i]].coordinates, nodes[tri[next]].coordinates);

            builder.add_edge(tri[i], tri[next], edge);
            builder.add_edge(tri[next], tri[i], edge);
        }
    }

    auto adj_list = Graph::adjacency_list_type::make_bidirectional_undirected(builder.get());
    return Graph::make_graph(std::move(nodes), std::move(adj_list));
}

steiner_graph triangulation_file_io::read_steiner(std::istream &input, float __epsilon) {
    using f = stream_encoders::encode_text;
    f::skip_comments(input);

    size_t node_count(f::template read<size_t>(input));
    size_t triangle_count(f::template read<size_t>(input));

    // read nodes
    std::vector<steiner_graph::triangle_node_info_type> nodes;
    for (int i = 0; i < node_count; ++i) {
        node_t n{};
        n.coordinates = f::template read<coordinate_t>(input);
        nodes.push_back(n);
    }

    // build adjacency list
    typename unidirectional_adjacency_list<node_id_t, std::nullptr_t>::adjacency_list_builder builder(node_count);

    // read triangles and generate edges from them
    std::vector<std::array<node_id_t, 3>> faces;
    for (int t = 0; t < triangle_count; t++) {
        triangle tri = f::template read<triangle>(input);
        for (int i = 0; i < 3; ++i) {
            auto next = (i + 1) % 3;
            builder.add_edge(tri[i], tri[next], 0);
            builder.add_edge(tri[next], tri[i], 0);
        }
        faces.push_back(tri);
    }

    auto adj_list = steiner_graph::adjacency_list_type::make_bidirectional_undirected(builder.get());
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
                auto n0 = graph.base_graph().source(edges[0]);
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
