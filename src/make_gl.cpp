#include <filesystem>
#include <fstream>
#include <string>

#include "file-io/file_io_impl.h"
#include "file-io/gl_file_io.h"
#include "file-io/gl_file_io_impl.h"
#include "file-io/triangulation_file_io.h"
#include "file-io/triangulation_file_io_impl.h"
#include "routing_impl.h"
#include "triangulation/node_properties.h"

struct edge_properties {
    bool is_boundary_edge;
};

int
main(int argc, char const *argv[]) {
    std::filesystem::path filename;
    std::filesystem::path filename_out;
    if (argc > 1)
        filename = std::string(argv[1]);
    else {
        std::cout << "input filename: " << std::flush;
        std::cin >> filename;
    }

    if (argc > 2)
        filename_out = std::string(argv[2]);
    else {
        std::cout << "output filename: " << std::flush;
        std::cin >> filename_out;
    }

    int color, boundary_color, linewidth;
    if (argc > 3)
        linewidth = std::stoi(argv[3]);
    else {
        std::cout << "output linewidth: " << std::flush;
        std::cin >> linewidth;
    }

    if (argc > 4)
        color = std::stoi(argv[4]);
    else {
        std::cout << "output color: " << std::flush;
        std::cin >> color;
    }
    boundary_color = color + 1;


    // read graph
    std::ifstream input(filename, std::ios::in);
    std::ofstream output(filename_out, std::ios::out);

    auto const& input_file_ending = filename.extension();
    auto const& output_file_ending = filename_out.extension();

    if (input_file_ending == ".graph") {
        std::vector<node_t> nodes;
        std::vector<std::array<node_id_t, 3>> faces;

        long node_count, face_count;
        input >> node_count >> face_count;

        nodes.resize(node_count);
        faces.resize(face_count);

        file_io::read_nodes<node_t, stream_encoders::encode_text>(input, nodes);
        file_io::read_triangles<node_id_t, stream_encoders::encode_text>(input, faces);

        std::unordered_map<std::pair<long,long>, edge_properties> properties;
        compute_boundary_edge(nodes, faces, properties);

        unidirectional_adjacency_list<node_id_t, gl_edge_t>::adjacency_list_builder builder;
        builder.add_edges_from_triangulation(faces);
        builder.finalize();
        auto edges = builder.edges();

        // set color and line width
        for(auto& edge : edges) {
            edge.info.color = (properties.contains(std::minmax({edge.source, edge.destination})) && properties[std::minmax({edge.source, edge.destination})].is_boundary_edge) ? boundary_color : color;
            edge.info.line_width = linewidth;
        }

        if (output_file_ending == ".gl") {
            output << node_count << '\n';
            output << edges.size() << '\n';
            file_io::write_nodes<node_t>(output, {nodes.begin(), nodes.end()});
            file_io::write_edges(output, edges);
        }
    } else if (input_file_ending == ".fmi") {
        std::vector<node_t> nodes;
        std::vector<adjacency_list_edge<node_id_t, edge_t>> edges;

        long node_count, edge_count;
        input >> node_count >> edge_count;

        nodes.resize(node_count);
        edges.resize(edge_count);

        file_io::read_nodes<node_t, stream_encoders::encode_text>(input, nodes);
        file_io::read_edges<adjacency_list_edge<node_id_t, edge_t>, stream_encoders::encode_text>(input, edges);

        unidirectional_adjacency_list<node_id_t, gl_edge_t>::adjacency_list_builder builder;
        for (auto&& edge : edges) {
            builder.add_edge(edge.source, edge.destination, gl_edge_t{color, linewidth});
        }

        output << node_count << '\n';
        output << edges.size() << '\n';
        file_io::write_nodes<node_t>(output, {nodes.begin(), nodes.end()});
        file_io::write_edges(output, builder.edges());
    }

    output.close();
}