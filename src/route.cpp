#include "interface/Client.h"

#include "cli/cmdline_route.h"

#include <fstream>
#include <sstream>

#include <filesystem>
#include <string>


int
main(int argc, char *argv[]) {
    gengetopt_args_info arguments;

    int status = cmdline_parser(argc, argv, &arguments);
    if (status != 0) {
        return 1;
    }

    std::filesystem::path graph_file;
    std::filesystem::path output_directory;
    double epsilon = arguments.epsilon_arg;

    graph_file = arguments.graph_file_arg;
    output_directory = arguments.output_directory_arg;

    // setup configuration
    RoutingConfiguration config;
    config.store_coords = arguments.coords_explicit_flag;
    config.only_distance = arguments.no_tree_flag;
    config.use_a_star = arguments.astar_flag;
    config.live_status = arguments.live_status_flag;

    // TODO: move to Client.cpp somehow
    static const std::unordered_map<enum_neighbor_finding, RoutingConfiguration::NeighborFindingAlgorithm> algorithms = {
            {neighbor_finding_arg_param,  RoutingConfiguration::NeighborFindingAlgorithm::PARAM},
            {neighbor_finding_arg_trig,   RoutingConfiguration::NeighborFindingAlgorithm::ATAN2},
            {neighbor_finding_arg_binary, RoutingConfiguration::NeighborFindingAlgorithm::BINSEARCH},
            {neighbor_finding_arg_linear, RoutingConfiguration::NeighborFindingAlgorithm::LINEAR},
            {neighbor_finding__NULL,      RoutingConfiguration::NeighborFindingAlgorithm::PARAM}
    };

    static const std::unordered_map<enum_pruning, RoutingConfiguration::Pruning> pruning = {
            {pruning__NULL,                         RoutingConfiguration::Pruning::PRUNE_DEFAULT},
            {pruning_arg_none,                      RoutingConfiguration::Pruning::UNPRUNED},
            {pruning_arg_prune,                     RoutingConfiguration::Pruning::PRUNE_DEFAULT},
            {pruning_arg_pruneMINUS_minMINUS_angle, RoutingConfiguration::Pruning::MinBendingAngleESpanner}
    };

    config.neighbor_selection_algorithm = algorithms.at(arguments.neighbor_finding_arg);
    config.pruning = pruning.at(arguments.pruning_arg);

    config.tree_size = (arguments.tree_given) ? arguments.tree_arg : 0;

    // read graph
    Client client;
    client.configure(config);
    if (arguments.epsilon_given && graph_file.extension() == ".graph")
        client.read_graph_file(graph_file, epsilon);
    else
        client.read_graph_file(graph_file);

    client.write_graph_stats(std::cout);
    std::cout << "configuration: epsilon = " << epsilon << ", A* = " << config.use_a_star << ", neighbor finding = "
              << (int) config.neighbor_selection_algorithm << ", pruning = " << (int) config.pruning << '\n';

    bool from_stdin = (arguments.query_given < 1);
    std::size_t query_index = 0;

    while (true) {
        // get query
        long src_node{0};
        long dest_node{0};

        if (query_index + 1 < arguments.query_given) {
            src_node = std::stoi(arguments.query_arg[query_index++]);
            dest_node = std::stoi(arguments.query_arg[query_index++]);
        } else if (query_index < arguments.query_given) {
            src_node = std::stoi(arguments.query_arg[query_index++]);
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

        // setup writers for graphs to show
        std::stringstream target_dir_builder;
        target_dir_builder << output_directory.string() << "/" << src_node << "_" << dest_node;

        std::string target_directory = target_dir_builder.str();
        std::string beeline_file = target_directory + "/beeline.gl";
        std::string route_file = target_directory + "/path.gl";
        std::string tree_file = target_directory + "/tree.gl";
        std::string info_file = target_directory + "/info.csv";
        std::filesystem::create_directories(target_directory);
        std::ofstream output_beeline(beeline_file);
        std::ofstream output_route(route_file);
        std::ofstream output_tree(tree_file);
        std::ofstream output_info(info_file);

        if (arguments.query_given >= 2)
            std::cout << "computing route " << (query_index / 2) << " of " << (arguments.query_given / 2) << ": "
                      << src_node << ',' << dest_node << "\n";

        if (dest_node >= 0)
            client.compute_route(src_node, dest_node);
        else
            client.compute_one_to_all(src_node);

        client.write_query(std::cout);
        client.write_csv_header(output_info);
        client.write_csv(output_info);

        client.write_info(std::cout);
        std::cout << std::endl;

        if (arguments.projection_arg == enum_projection::projection_arg_google_bing) {
            client.query().beeline().project(Projection::WGS84_TO_GB);
            if (config.tree_size) {
                client.result().tree_forward().project(Projection::WGS84_TO_GB);
            }
        } else if (arguments.projection_arg == enum_projection::projection_arg_wgs84) {
            client.query().beeline().project(Projection::GB_TO_WGS84);
            if (config.tree_size) {
                client.result().tree_forward().project(Projection::GB_TO_WGS84);
            }
        }
        client.write_beeline_file(beeline_file);

        if (config.tree_size)
            client.write_tree_file(tree_file);

        if (!client.result().route_found())
            continue;

        if (arguments.projection_arg == enum_projection::projection_arg_google_bing) {
            client.result().path().project(Projection::WGS84_TO_GB);
        } else if (arguments.projection_arg == enum_projection::projection_arg_wgs84) {
            client.result().path().project(Projection::GB_TO_WGS84);
        }

        client.write_route_file(route_file);
    }

    return 0;
}
