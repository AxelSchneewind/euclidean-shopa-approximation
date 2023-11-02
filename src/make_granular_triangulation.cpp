#include "routing.h"

#include "file-io/triangulation_file_io.h"
#include "file-io/gl_file_io.h"
#include "file-io/gl_file_io_impl.h"

#include "triangulation/steiner_graph.h"
#include <fstream>

int main(int argc, const char **argv) {
    std::ifstream input(argv[1]);
    std::ofstream output(std::string(argv[1]) + ".gl");

    auto graph = triangulation_file_io::template read<steiner_graph>(input);

    gl_file_io::write_steiner(output, graph);

    output.close();
    return 0;
}