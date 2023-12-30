#include "interface/Client.h"

#include <fstream>
#include <chrono>

int main(int argc, const char *argv[]) {
    if (argc < 5)
        return 1;

    std::string graph_file(argv[1]);
    std::string output_file(argv[2]);

    double epsilon = std::stof(argv[3]);

    Client client;
    client.read_graph_file(graph_file, epsilon, false);

    client.write_graph_stats(std::cout);

    coordinate_t bottom_left { std::stod(argv[4]), std::stod(argv[5]) } ;
    coordinate_t top_right { std::stod(argv[6]), std::stod(argv[7]) } ;

    if (output_file.ends_with(".gl"))
        client.write_subgraph_file(output_file, bottom_left, top_right);
    else
        client.write_subgraph_file(output_file, bottom_left, top_right);
}
