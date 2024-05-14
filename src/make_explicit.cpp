#include "interface/Client.h"

#include <string>

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " path/to/graph path/to/output epsilon\n";
        return 1;
    }

    std::string const graph_file { argv[1] };
    std::string const graph_file_out { argv[2] };
    double const epsilon { std::stod(argv[3]) };

    std::cout << "making explicit graph " << graph_file_out << " by placing steiner points on triangulation " << graph_file << " with epsilon=" << epsilon << "\n";

    Client client;
    client.read_graph_file(graph_file, epsilon);
    client.write_graph_file(graph_file_out);
}