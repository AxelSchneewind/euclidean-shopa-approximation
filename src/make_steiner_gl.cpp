#include "routing.h"

#include "file-io/triangulation_file_io.h"
#include "file-io/gl_file_io.h"
#include "file-io/gl_file_io_impl.h"

#include "triangulation/steiner_graph.h"
#include <fstream>

int main(int argc, const char **argv) {
    std::string filename;
    if (argc <= 1)
        std::cin >> filename;
    else
        filename = argv[1];

    std::ifstream input(filename);
    std::ofstream output(filename + ".gl");

    auto graph = triangulation_file_io::read_steiner(input);

    gl_file_io::write_steiner(output, graph);

    output.close();
    return 0;
}