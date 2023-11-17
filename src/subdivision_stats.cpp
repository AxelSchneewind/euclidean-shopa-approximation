#include "steiner_routing.h"
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <ios>
#include <iomanip>
#include "util/memory_usage.h"
#include "triangulation/subdivision_table.h"


int
main(int argc, char const *argv[]) {

    float __epsilon = 0.50F;

    std::cout << "epsilon: " << std::flush;
    std::cin >> __epsilon;

    auto table = subdivision_table::precompute(__epsilon, 0.05);

    std::cout << "using epsilon = " << __epsilon << " \n";
    int index = 0;
    for (auto triangle_class : table) {
        std::cout << "for edges with angle ~" << std::setw(10) << subdivision_table::class_angle(index) << ": " << triangle_class.node_positions.size() << " points: ";
        std::cout << '\n';//"\b\b\n";

        index++;
    }
    std::cout << std::flush;

    // read graph
    std::string filename;
    std::string filename_out;
    std::cout << "input filename: " << std::flush;
    std::cin >> filename;
    std::cout << "output filename (N if no output should be generated): " << std::flush;
    std::cin >> filename_out;
    std::ifstream input(filename);
    std::ofstream output(filename_out);

    auto graph = triangulation_file_io::read_steiner(input, __epsilon);
    input.close();
    std::cout << "graph has " << graph.node_count()<<  " nodes and " << graph.edge_count() << " edges" << std::endl;


    // write gl file for graph
    if (!filename_out.empty() && (filename != "N" || filename != "n")) {
        std::cout << "writing, expected file size: " << graph.node_count() * 42 + graph.edge_count() * 8 << " bytes" << std::endl;
        gl_file_io::write<steiner_graph>(output, graph, 1, 1);
        output.close();
    }
}
