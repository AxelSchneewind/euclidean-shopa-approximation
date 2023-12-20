#include "interface/Client.h"

#include <fstream>
#include <chrono>

int main(int argc, const char *argv[]) {
    if (argc < 5)
        return 1;

    std::string graph_file(argv[1]);
    std::string output_file(argv[2]);
    std::ofstream output(output_file);
    float epsilon = std::stof(argv[3]);
    Client client;
    client.read_graph_file(graph_file, epsilon, false);

    client.write_graph_stats(std::cout);

    steiner_graph::triangle_node_id_type src = std::stoi(argv[4]);

    output << "node,distance\n";

    // init timing
    auto before
            = std::chrono::high_resolution_clock::now();

    client.compute_one_to_all(src, output);

    auto after
            = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> duration = after - before;

    std::cout << "Time: " << duration << std::endl;

    output.close();
}
