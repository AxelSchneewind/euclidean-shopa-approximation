#include "routing.h"

#include <format>
#include <fstream>

#include <filesystem>

int
main(int argc, char const *argv[]) {
    std::string graph_file;
    std::string output_directory;
    float epsilon = 0.5;

    if (argc > 3) {
        graph_file = argv[1];
        output_directory = argv[2];
        epsilon = std::stof(argv[3]);
    } else {
        std::cout << "graph file: ";
        std::cin >> graph_file;

        std::cout << "output directory: ";
        std::cin >> output_directory;

        std::cout << "epsilon: ";
        std::cin >> epsilon;
    }


    // read triangulation
    std::cout << "reading graph from " << graph_file << "... \n" << std::flush;

    // read graph
    Client client;
    client.read_graph_file(graph_file, epsilon);

    bool done = false;
    while (!done) {
        // get query
        int src_node(0);
        int dest_node(0);
        char mode = 'A';

        if (argc > 5) {
            src_node = std::stoi(argv[4]);
            dest_node = std::stoi(argv[5]);
            done = true;
        } else {
            std::cout << "src node: " << std::flush;
            std::cin >> src_node;
            std::cout << "dest node: " << std::flush;
            std::cin >> dest_node;

            std::cout << "mode (B = Bidirectional Dijkstra, A = A*) : A" << std::flush;
            //std::cin >> mode;
            std::cout << std::endl;
        }

        // setup writer for graphs to show
        std::string target_directory = std::format("{}/{}_{}_{}_{}", output_directory, mode, src_node, dest_node, (int)(epsilon * 10));
        std::filesystem::create_directory(target_directory);
        std::string beeline_file = std::format("{}/beeline.gl", target_directory);
        std::string route_file = std::format("{}/route.gl", target_directory);
        std::string tree_file = std::format("{}/tree.gl", target_directory);
        std::string info_file = std::format("{}/info.gl", target_directory);
        std::ofstream output_beeline(beeline_file);
        std::ofstream output_route(route_file);
        std::ofstream output_tree(tree_file);
        std::ofstream output_info(info_file);


        std::cout << "computing route..." << std::flush;

        client.compute_route(src_node, dest_node);

        client.write_info(std::cout);

        std::cout << "saving results to " << target_directory << std::flush;

        client.write_beeline_file(output_beeline);
        client.write_route_file(output_route);
        client.write_tree_file(output_tree);

        output_route.close();
        output_beeline.close();
        output_tree.close();
        output_info.close();
    }

    return 0;
}
