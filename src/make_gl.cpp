#include <fstream>
#include <string>

#include "file-io/file_io_impl.h"
#include "file-io/gl_file_io.h"
#include "file-io/gl_file_io_impl.h"
#include "file-io/triangulation_file_io.h"
#include "file-io/triangulation_file_io_impl.h"
#include "routing_impl.h"

template<RoutableGraph G, typename file_io_in, typename file_io_out>
void make_gl(std::istream &input, std::ostream &output, int linewidth, int color) {
    using f_in = file_io_in;
    using f_out = file_io_out;

    // read
    G graph = f_in::template read<G>(input);

    std::cout << "read graph with " << graph.node_count() << " nodes and " << graph.edge_count() / 2 << " edges"
              << std::endl;

    // write gl file for graph
    f_out::template write<G>(output, graph, linewidth, color);
}

template<typename file_io_in, typename file_io_out>
void make_steiner_gl(std::istream &input, std::ostream &output, int linewidth, int color, float epsilon) {
    using f_in = file_io_in;
    using f_out = file_io_out;

    // read
    steiner_graph graph = f_in::read_steiner(input, epsilon);

    std::cout << "read graph with " << graph.node_count() << " nodes and " << graph.edge_count() / 2 << " edges "
              << "from which " << graph.base_graph().node_count() << " nodes and " << graph.base_graph().edge_count() / 2 << " are stored explicitly"
              << std::endl;

    // write gl file for graph
    f_out::template write<steiner_graph>(output, graph, linewidth, color);
}


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

    float epsilon;
    if (argc > 3)
        epsilon = std::stof(argv[3]);
    else {
        std::cout << "epsilon: ";
        std::cin >> epsilon;
    }

    int color, linewidth;
    if (argc > 4)
        linewidth = std::stoi(argv[4]);
    else {
        std::cout << "output linewidth: " << std::flush;
        std::cin >> linewidth;
    }

    if (argc > 5)
        color = std::stoi(argv[5]);
    else {
        std::cout << "output color: " << std::flush;
        std::cin >> color;
    }


    // read graph
    std::ifstream input(filename, std::ios::in);
    std::ofstream output(filename_out, std::ios::out);

    std::string_view input_file_ending(&filename.at(filename.find_first_of('.')));
    std::string_view output_file_ending(&filename_out.at(filename_out.find_first_of('.')));

    if (input_file_ending == ".graph") {
        std::vector<coordinate_t> nodes;
        std::vector<std::array<unsigned long, 3>> faces;

        long node_count, face_count;
        input >> node_count >> face_count;

        nodes.resize(node_count);
        faces.resize(face_count);

        file_io::read_nodes<coordinate_t, stream_encoders::encode_text>(input, nodes);
        file_io::read_triangles<unsigned long, stream_encoders::encode_text>(input, faces);

        unidirectional_adjacency_list<unsigned long, gl_edge_t>::adjacency_list_builder builder;
        builder.add_edges_from_triangulation(faces);
        builder.sort_edges();
        auto edges = builder.edges();

        // set color and linewidth
        for(auto& edge : edges) {
            edge.info.color = color;
            edge.info.line_width = linewidth;
        }

        if (output_file_ending == ".steiner.gl") {
            make_steiner_gl<triangulation_file_io, gl_file_io>(input, output, linewidth, color, epsilon);
        } else if (output_file_ending == ".gl") {
            output << node_count << std::endl;
            output << edges.size() << std::endl;
            file_io::write_nodes(output, std::span<coordinate_t>{nodes.begin(), nodes.end()});
            file_io::write_edges(output, edges);
        }
    }

    output.close();
}