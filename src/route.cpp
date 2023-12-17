#include "routing.h"

#include "cli/cmdline_route.h"

#include <format>
#include <fstream>

#include <filesystem>


int
main(int argc, char const *argv[]) {
    gengetopt_args_info arguments;

    int status = cmdline_parser(argc, (char **) argv, &arguments);
    if (status != 0) {
        return 1;
    }

    if (arguments.help_given != 0) {
        std::cout << arguments.help_help << std::endl;
        return 0;
    }


    std::string graph_file;
    std::string output_directory;
    double epsilon = 0.5;
    bool output_csv = arguments.csv_format_flag != 0;

    graph_file = arguments.graph_file_arg;
    output_directory = arguments.output_directory_arg;
    epsilon = arguments.epsilon_arg;

    // read graph
    Client client;
    if (graph_file.ends_with(".graph"))
        client.read_graph_file(graph_file, (float)epsilon, output_csv);
    else
        client.read_graph_file(graph_file, output_csv);


    if (output_csv)
        client.write_csv_header(std::cout);
    else client.write_graph_stats(std::cout);

    bool done = false;
    bool from_stdin = (arguments.query_given < 1) || (arguments.stdin_flag != 0);
    int query_index = 0;

    while (!done) {
        // get query
        int src_node(0);
        int dest_node(0);

        if (query_index + 1 < arguments.query_given) {
            src_node = arguments.query_arg[query_index++];
            dest_node = arguments.query_arg[query_index++];
        } else if (query_index < arguments.query_given) {
            src_node = arguments.query_arg[query_index++];
            dest_node = -1;
        } else if (from_stdin) {
            std::cout << "src node: " << std::flush;
            std::cin >> src_node;
            std::cout << "dest node: " << std::flush;
            std::cin >> dest_node;
            std::cout << std::endl;
        } else {
            break;
        }

        // setup writer for graphs to show
        std::string eps_string = epsilon == 0 ? "exact" : std::format("{:_>5d}", (int) (epsilon * 10000));
        std::string target_directory = std::format("{}/{}_{}_{}", output_directory, src_node, dest_node, eps_string);
        std::filesystem::create_directory(target_directory);
        std::string beeline_file = std::format("{}/beeline.gl", target_directory);
        std::string route_file = std::format("{}/route.gl", target_directory);
        std::string tree_file = std::format("{}/tree.gl", target_directory);
        std::string info_file = std::format("{}/info.csv", target_directory);
        std::ofstream output_beeline(beeline_file);
        std::ofstream output_route(route_file);
        std::ofstream output_tree(tree_file);
        std::ofstream output_info(info_file);

        if (dest_node >= 0)
            client.compute_route(src_node, dest_node);
        else
            client.compute_one_to_all(src_node);

        if (output_csv)
            client.write_csv(std::cout);
        else {
            client.write_query(std::cout);
            client.write_info(std::cout);
        }

        client.write_csv_header(output_info);
        client.write_csv(output_info);
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
