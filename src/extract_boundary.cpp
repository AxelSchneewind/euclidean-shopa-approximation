#include "triangulation/node_properties.h"

#include "file-io/file_io.h"
#include "file-io/file_io_impl.h"

#include "graph/geometry.h"
#include "graph/geometry_impl.h"
#include "graph/adjacency_list.h"
#include "graph/adjacency_list_impl.h"
#include "graph/unidirectional_adjacency_list.h"
#include "graph/unidirectional_adjacency_list_impl.h"

#include <fstream>

int main(int argc, char *argv[]) {
    if (argc < 3) {
        std::cout << "usage: "<< argv[0] << " path/to/input.graph path/to/output.fmi\n";
        return 1;
    }

    std::string graph_file(argv[1]);
    std::string output_file(argv[2]);

    std::ifstream input(graph_file);
    std::ofstream output(output_file);

    if (graph_file.ends_with(".graph") && output_file.ends_with(".fmi")) {
        std::size_t node_count, face_count;
        input >> node_count;
        input >> face_count;

        std::vector<node_t> nodes(node_count);
        file_io::read_nodes<node_t>(input, { nodes.begin(), nodes.end() });

        std::vector<std::array<node_id_t, 3>> triangles(face_count);
        file_io::read_triangles<node_id_t>(input, {triangles.begin(), triangles.end()});

        struct edge_property { bool is_boundary_edge; };
        std::unordered_map<std::pair<long, long>, edge_property> properties;
        compute_boundary_edge(nodes, triangles, properties);

        // add edges that belong to boundary
        std::unordered_map<node_id_t, node_id_t> new_id;
        std::vector<node_t> nodes_out;
        adjacency_list<node_id_t, edge_t>::builder edges;
        for (auto const& triangle : triangles) {
            for (int i = 0; i < 3; ++i) {
                long node1 = triangle[i], node2 = triangle[(i + 1) % 3];

                if (properties.contains({node1, node2}) && properties[{node1, node2}].is_boundary_edge) {
                    if (!new_id.contains(node1)) {
                        new_id[node1] = nodes_out.size();
                        nodes_out.emplace_back(nodes[node1]);
                    }
                    if (!new_id.contains(node2)) {
                        new_id[node2] = nodes_out.size();
                        nodes_out.emplace_back(nodes[node2]);
                    }

                    edges.add_edge(new_id[node1], new_id[node2], distance(nodes[node1].coordinates, nodes[node2].coordinates));
                    edges.add_edge(new_id[node2], new_id[node1], distance(nodes[node1].coordinates, nodes[node2].coordinates));
                }
            }
        }
        edges.finalize();

        // write graph file
        output << nodes_out.size() << '\n' << edges.edges().size() <<'\n';
        file_io::write_nodes<node_t>(output, nodes_out);
        file_io::write_edges(output, edges.edges());
    }
}
