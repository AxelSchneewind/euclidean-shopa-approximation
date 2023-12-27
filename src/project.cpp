#include "interface/Graph.h"

#include <fstream>



int main(int argc, const char* argv[]) {
    std::cout << "Reading graph from " << argv[1] << "..." << std::flush;
    Graph graph;
    graph.read_graph_file(argv[1]);

    std::cout << "\n, done" << std::endl;

    Projection projection = Projection::NONE;
    std::string projection_name(argv[3]);
    if (projection_name == "xy_to_lat_long")
        projection = Projection::GB_TO_WGS84;
    if (projection_name == "lat_long_to_xy")
        projection = Projection::WGS84_TO_GB;

    std::cout << "projecting with " << (int)projection << "..." << std::flush;
    graph.project(projection);

    std::cout << "\b\b\b, done" << std::endl;

    std::cout << "writing to " << argv[2] << "..." << std::flush;
    graph.write_graph_file(argv[2]);
    std::cout << "\b\b\b, done" << std::endl;
}