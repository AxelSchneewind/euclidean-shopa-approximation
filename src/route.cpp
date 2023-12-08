#include "routing.h"

#include "cli/cmdline_route_interactive.h"

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


    // read graph
    Client client;
    if (graph_file.ends_with(".graph"))
        client.read_graph_file(graph_file, epsilon, true);
    else
        client.read_graph_file(graph_file, true);

    client.write_graph_stats(std::cout);

    bool done = false;
    bool from_stdin = (argc < 6);

    int source_dest_index = 4;
    while (!done) {
        // get query
        int src_node(0);
        int dest_node(0);

        if (!from_stdin) {
            src_node = std::stoi(argv[source_dest_index++]);
            dest_node = std::stoi(argv[source_dest_index++]);
            if (argc <= source_dest_index + 1)
                done = true;
        } else {
            std::cout << "src node: " << std::flush;
            std::cin >> src_node;
            std::cout << "dest node: " << std::flush;
            std::cin >> dest_node;
            std::cout << std::endl;
        }

        // setup writer for graphs to show
        std::string target_directory = std::format("{}/{}_{}_{}", output_directory, src_node, dest_node,
                                                   (int) (epsilon * 100));
        std::filesystem::create_directory(target_directory);
        std::string beeline_file = std::format("{}/beeline.gl", target_directory);
        std::string route_file = std::format("{}/route.gl", target_directory);
        std::string tree_file = std::format("{}/tree.gl", target_directory);
        std::string info_file = std::format("{}/info.gl", target_directory);
        std::ofstream output_beeline(beeline_file);
        std::ofstream output_route(route_file);
        std::ofstream output_tree(tree_file);
        std::ofstream output_info(info_file);

        client.write_graph_stats(output_info);

        client.compute_route(src_node, dest_node);

        client.write_query(std::cout);
        client.write_query(output_info);

        client.write_info(std::cout);
        client.write_info(output_info);

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
