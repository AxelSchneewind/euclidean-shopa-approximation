#include <fstream>
#include <string>

#include "file-io/gl_file_io.h"
#include "file-io/gl_file_io_impl.h"
#include "file-io/triangulation_file_io.h"
#include "file-io/triangulation_file_io_impl.h"
#include "routing.h"

template<RoutableGraph G, typename file_io_in, typename file_io_out>
void make_gl(std::istream &input, std::ostream &output) {
    using f_in = file_io_in;
    using f_out = file_io_out;

    // read
    G graph = f_in::template read<G>(input);

    // write gl file for graph
    f_out::template write<G>(output, graph, 2, 2);
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
    std::ifstream input(filename);
    std::ofstream output(filename_out);

    std::string_view input_file_ending(&filename.at(filename.find_first_of('.')));
    std::string_view output_file_ending(&filename_out.at(filename_out.find_first_of('.')));

    if (input_file_ending == ".graph") {
        if (output_file_ending == ".steiner.gl")
            make_gl<steiner_graph, triangulation_file_io, gl_file_io>(input, output);
        else if (output_file_ending == ".gl")
            make_gl<std_graph_t, triangulation_file_io, gl_file_io>(input, output);
    }

    output.close();
}