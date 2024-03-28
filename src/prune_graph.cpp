#include "file-io/file_io.h"
#include "file-io/file_io_impl.h"

#include "graph/geometry.h"
#include "graph/geometry_impl.h"
#include "graph/adjacency_list.h"
#include "graph/unidirectional_adjacency_list.h"

#include "cli/cmdline_prune_graph.h"

#include <fstream>
#include <chrono>

int main(int argc, char *argv[]) {
    gengetopt_args_info args;
    int ret = cmdline_parser(argc, argv, &args);

    if (ret != 0 || args.help_given) {
        cmdline_parser_print_help();
        return 0;
    }

    std::string graph_file(args.graph_file_arg);
    std::string output_file(args.output_file_arg);

    std::ifstream input(graph_file);
    std::ofstream output(output_file);

    coordinate_t bottom_left{args.minY_arg, args.minX_arg };
    coordinate_t top_right{args.maxY_arg, args.maxX_arg};

    if (graph_file.ends_with(".graph") && output_file.ends_with(".graph")) {
        std::size_t node_count, face_count;
        input >> node_count;
        input >> face_count;

        std::vector<node_t> nodes(node_count);
        file_io::read_nodes<node_t>(input, { nodes.begin(), nodes.end() });

        std::vector<std::array<node_id_t, 3>> triangles(face_count);
        file_io::read_triangles<node_id_t>(input, {triangles.begin(), triangles.end()});

        // find triangles in selected area and their respective nodes
        std::unordered_map<int, bool> contained;
        face_count = 0;
        for (size_t i = 0; i < triangles.size(); ++i) {
            auto& triangle = triangles[i];
            if (is_in_rectangle(nodes[triangle[0]].coordinates, bottom_left, top_right) ||
                is_in_rectangle(nodes[triangle[1]].coordinates, bottom_left, top_right) ||
                is_in_rectangle(nodes[triangle[2]].coordinates, bottom_left, top_right)) {

                contained[triangle[0]] = true;
                contained[triangle[1]] = true;
                contained[triangle[2]] = true;

                triangles[face_count++] = triangle;
            }
        }
        triangles.resize(face_count);
        triangles.shrink_to_fit();

        // assign new node ids
        std::unordered_map<node_id_t, node_id_t> new_node_id;
        node_count = 0;
        for (size_t id = 0; id < nodes.size(); id++) {
            if (contained.contains(id) && contained[id])
                new_node_id[id] = node_count++;
        }
        contained.clear();

        // apply new node ids
        for (auto & face : triangles) {
            assert(new_node_id.contains(face[0]) && new_node_id.contains(face[1]) && new_node_id.contains(face[2]));

            face[0] = new_node_id[face[0]];
            face[1] = new_node_id[face[1]];
            face[2] = new_node_id[face[2]];
        }

        // reorder node information
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (new_node_id.contains(i)) {
                assert(static_cast<size_t>(new_node_id[i]) <= i);
                nodes.at(new_node_id[i]) = nodes[i];
            }
        }
        nodes.resize(node_count);
        nodes.shrink_to_fit();
        new_node_id.clear();

        // write pruned graph file
        output << node_count << '\n' << face_count <<'\n';
        file_io::write_nodes<node_t>(output, {nodes.begin(), nodes.end()});
        file_io::write_triangles<node_id_t>(output, {triangles.begin(), triangles.end()});
    }
    if (graph_file.ends_with(".fmi")) {
        std::size_t node_count, edge_count;
        input >> node_count;
        input >> edge_count;

        std::vector<node_t> nodes(node_count);
        file_io::read_nodes<node_t>(input, { nodes.begin(), nodes.end() });

	using edge_type = adjacency_list_edge<node_id_t, edge_t>;
        std::vector<edge_type> edges(edge_count);
        file_io::read_edges<edge_type>(input, {edges.begin(), edges.end()});

        // find edges in selected area and their respective nodes
        std::unordered_map<int, bool> contained;
        edge_count = 0;
        for (size_t i = 0; i < edges.size(); ++i) {
            if (is_in_rectangle(nodes[edges[i].source].coordinates, bottom_left, top_right) &&
                is_in_rectangle(nodes[edges[i].destination].coordinates, bottom_left, top_right)) {

                contained[edges[i].source] = true;
                contained[edges[i].destination] = true;

                edges[edge_count++] = edges[i];
            }
        }
        edges.resize(edge_count);
        edges.shrink_to_fit();

        // assign new node ids
        std::unordered_map<node_id_t, node_id_t> new_node_id;
        node_count = 0;
        for (size_t id = 0; id < nodes.size(); id++) {
            if (contained.contains(id) && contained[id])
                new_node_id[id] = node_count++;
        }
        contained.clear();

        // apply new node ids
        for (auto &edge : edges) {
            assert(new_node_id.contains(edge.source) && new_node_id.contains(edge.destination));

            edge.source = new_node_id[edge.source];
            edge.destination = new_node_id[edge.destination];
        }

        // reorder node information
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (new_node_id.contains(i)) {
                assert(static_cast<size_t>(new_node_id[i]) <= i);
                nodes.at(new_node_id[i]) = nodes[i];
            }
        }
        nodes.resize(node_count);
        nodes.shrink_to_fit();
        new_node_id.clear();

        // write pruned graph file
        output << node_count << '\n' << edge_count <<'\n';
        file_io::write_nodes<node_t>(output, {nodes.begin(), nodes.end()});
        file_io::write_edges<edge_type>(output, {edges.begin(), edges.end()});
    }
}
