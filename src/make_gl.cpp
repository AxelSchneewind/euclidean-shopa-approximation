#include <fstream>
#include <string>

#include "file-io/file_io_impl.h"
#include "file-io/gl_file_io.h"
#include "file-io/gl_file_io_impl.h"
#include "file-io/triangulation_file_io.h"
#include "file-io/triangulation_file_io_impl.h"
#include "routing_impl.h"

int
main(int argc, char const *argv[]) {
    std::string filename;
    std::string filename_out;
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

    int color, linewidth;
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


    // read graph
    std::ifstream input(filename, std::ios::in);
    std::ofstream output(filename_out, std::ios::out);

    std::string_view input_file_ending(&filename.at(filename.find_last_of('.')));
    std::string_view output_file_ending(&filename_out.at(filename_out.find_last_of('.')));

    if (input_file_ending == ".graph") {
        std::vector<node_t> nodes;
        std::vector<std::array<node_id_t, 3>> faces;

        long node_count, face_count;
        input >> node_count >> face_count;

        nodes.resize(node_count);
        faces.resize(face_count);

        file_io::read_nodes<node_t, stream_encoders::encode_text>(input, nodes);
        file_io::read_triangles<node_id_t, stream_encoders::encode_text>(input, faces);

        unidirectional_adjacency_list<node_id_t, gl_edge_t>::adjacency_list_builder builder;
        builder.add_edges_from_triangulation(faces);
        builder.sort_edges();
        auto edges = builder.edges();

        // set color and line width
        for(auto& edge : edges) {
            edge.info.color = color;
            edge.info.line_width = linewidth;
        }

        if (output_file_ending == ".steiner.gl") {
            double epsilon;
            if (argc > 5)
                epsilon = std::stof(argv[3]);
            else {
                std::cout << "epsilon: ";
                std::cin >> epsilon;
            }

            unidirectional_adjacency_list<node_id_t>::adjacency_list_builder builder;
            builder.add_edges_from_triangulation(faces);
            auto steiner_edges = adjacency_list<node_id_t>::make_bidirectional(builder.get());
            auto g = steiner_graph::make_graph(std::move(nodes), std::move(steiner_edges), std::move(faces), epsilon);

            gl_file_io::write(output, g, linewidth, color);
        } else if (output_file_ending == ".gl") {
            output << node_count << '\n';
            output << edges.size() << '\n';
            file_io::write_nodes<node_t>(output, {nodes.begin(), nodes.end()});
            file_io::write_edges(output, edges);
        }
    }

    output.close();
}