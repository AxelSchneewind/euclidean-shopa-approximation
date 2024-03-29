
#include "interface/Client.h"
#include <string>




int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cout << "usage: " << argv[0] << " path/to/graph path/to/output epsilon\n";
        return 1;
    }

    std::string graph_file = argv[1];
    std::string graph_file_out = argv[2];
    double epsilon = std::stod(argv[3]);

    Client client;
    client.read_graph_file(graph_file, epsilon);
    client.write_graph_file(graph_file_out);
}