#include "interface/Client.h"

#include <fstream>
#include <chrono>
#include <ratio>
#include <string>

int main(int argc, const char *argv[]) {
    if (argc < 5) {
        std::cout << "usage ota path/to/graph path/to/output epsilon source-id\n";
        return 1;
    }

    std::string graph_file(argv[1]);
    std::string output_file(argv[2]);
    std::ofstream output(output_file);
    std::ofstream output_info(output_file + ".csv");
    double epsilon = std::stof(argv[3]);
    long src = std::stol(argv[4]);

    Client client;
    client.read_graph_file(graph_file, epsilon);
    client.write_graph_stats(std::cout);

    client.compute_one_to_all(src, output);

    client.write_query(std::cout);
    client.write_csv_header(output_info);
    client.write_csv(output_info);
}
