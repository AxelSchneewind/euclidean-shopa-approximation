#include <fstream>
#include <string>

#include "file-io/gl_file_io.h"
#include "file-io/gl_file_io_impl.h"
#include "file-io/triangulation_file_io.h"
#include "file-io/triangulation_file_io_impl.h"
#include "routing.h"

template<RoutableGraph G, typename file_io_in, typename file_io_out>
void make_gl(std::istream &input, std::ostream &output, int linewidth, int color) {
    using f_in = file_io_in;
    using f_out = file_io_out;

    // read
    G graph = f_in::template read<G>(input);

    std::cout << "read graph with " << graph.node_count() << " nodes and " << graph.edge_count() << " edges" << std::endl;

    // write gl file for graph
    f_out::template write<G>(output, graph, linewidth, color);
}
template<typename file_io_in, typename file_io_out>
void make_steiner_gl(std::istream &input, std::ostream &output, int linewidth, int color) {
    using f_in = file_io_in;
    using f_out = file_io_out;

    float epsilon = 0.5;
    std::cout << "epsilon: ";
    std::cin >> epsilon;

    // read
    steiner_graph graph = f_in::read_steiner(input, epsilon);

    std::cout << "read graph with " << graph.node_count() << " nodes and " << graph.edge_count() << " edges" << std::endl;

    // write gl file for graph
    f_out::template write<steiner_graph>(output, graph, linewidth, color);
}


int
main(int argc, char const *argv[]) {

    std::string filename;
    std::string filename_out;
    std::cout << "input filename: " << std::flush;
    std::cin >> filename;
    std::cout << "output filename: " << std::flush;
    std::cin >> filename_out;


    // read graph
    std::ifstream input(filename, std::ios::in);
    std::ofstream output(filename_out, std::ios::out);

    std::string_view input_file_ending(&filename.at(filename.find_first_of('.')));
    std::string_view output_file_ending(&filename_out.at(filename_out.find_first_of('.')));

    if (input_file_ending == ".graph") {

        if (output_file_ending == ".steiner.gl") {
            int color, linewidth;
            std::cout << "output linewidth: " << std::flush;
            std::cin >> linewidth;
            std::cout << "output color: " << std::flush;
            std::cin >> color;

            make_steiner_gl<triangulation_file_io, gl_file_io>(input, output, linewidth, color);
        }
        else if (output_file_ending == ".gl") {
            int color, linewidth;
            std::cout << "output linewidth: " << std::flush;
            std::cin >> linewidth;
            std::cout << "output color: " << std::flush;
            std::cin >> color;

            make_gl<std_graph_t, triangulation_file_io, gl_file_io>(input, output, color, linewidth);
        }
    }

    output.close();
}