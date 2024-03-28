#include "interface/Graph.h"

#include <fstream>



int main(int argc, const char* argv[]) {
    if (argc < 3) {
        std::cout << "usage " << argv[0] << " path/to/graph-file path/to/result [xy_to_latlon|latlon_to_xy]\n";
        return 1;
    }

    std::cout << "Reading graph from " << argv[1] << "...";
    Graph graph;
    graph.read_graph_file(argv[1]);
    std::cout << "\b\b\b, done" << std::endl;

    Projection projection = Projection::NONE;
    std::string projection_name(argv[3]);
    if (projection_name == "xy_to_latlon")
        projection = Projection::GB_TO_WGS84;
    if (projection_name == "latlon_to_xy")
        projection = Projection::WGS84_TO_GB;

    std::cout << "projecting with " << (int)projection << "...";
    graph.project(projection);

    std::cout << "\b\b\b, done" << std::endl;

    std::cout << "writing to " << argv[2] << "...";
    graph.write_graph_file(argv[2]);
    std::cout << "\b\b\b, done" << std::endl;
}