#include "routing.h"

#include <format>
#include <fstream>

#include <filesystem>

int
main(int argc, char const *argv[]) {
    std::string graph_file;
    std::string visibility_file;
    std::string output_directory;

    if (argc > 3) {
        graph_file = argv[1];
        visibility_file = argv[2];
        output_directory = argv[3];
    } else {
        std::cout << "graph file: ";
        std::cin >> graph_file;

        std::cout << "visibility edge file: ";
        std::cin >> graph_file;

        std::cout << "output directory: ";
        std::cin >> output_directory;
    }

    Client client;
    client.read_visibility_graph(graph_file, visibility_file);
    client.write_graph_stats(std::cout);

    bool done = false;
    while (!done) {
        // get query
        int src_node(0);
        int dest_node(0);

        if (argc > 5) {
            src_node = std::stoi(argv[4]);
            dest_node = std::stoi(argv[5]);
            done = true;
        } else {
            std::cout << "src node: " << std::flush;
            std::cin >> src_node;
            std::cout << "dest node: " << std::flush;
            std::cin >> dest_node;
        }

        // setup writer for graphs to show
        std::string target_directory = std::format("{}/results/{}_{}_{}", output_directory, src_node, dest_node, "exact");
        std::filesystem::create_directory(target_directory);
        std::string beeline_file = std::format("{}/beeline.gl", target_directory);
        std::string route_file = std::format("{}/route.gl", target_directory);
        std::string tree_file = std::format("{}/tree.gl", target_directory);
        std::string info_file = std::format("{}/info.gl", target_directory);
        std::ofstream output_beeline(beeline_file);
        std::ofstream output_route(route_file);
        std::ofstream output_tree(tree_file);
        std::ofstream output_info(info_file);

        client.compute_route(src_node, dest_node);
        client.write_info(std::cout);

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
