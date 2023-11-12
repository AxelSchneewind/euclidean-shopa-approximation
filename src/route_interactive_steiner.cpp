#include <format>
#include <fstream>

#include "file-io/fmi_file_io.h"
#include "file-io/gl_file_io.h"
#include "file-io/triangulation_file_io.h"
#include "graph/graph.h"
#include "triangulation/steiner_graph_impl.h"

#include "file-io/gl_file_io_impl.h"
#include "file-io/triangulation_file_io_impl.h"

#include "routing.h"
#include "query.h"

#include <chrono>
#include <filesystem>

int
main(int argc, char const *argv[]) {
    std::string graph_file;
    std::string output_directory;

    if (argc > 2) {
        graph_file = argv[1];
        output_directory = argv[2];
    } else {
        std::cout << "graph file: ";
        std::cin >> graph_file;

        std::cout << "output directory: ";
        std::cin >> output_directory;
    }


    // read triangulation
    std::cout << "reading graph from " << graph_file << "... \n" << std::flush;
    std::cout << "\tbtw: expected size per node: " << steiner_graph::SIZE_PER_NODE << " and per edge "
              << steiner_graph::SIZE_PER_EDGE << std::endl;

    std::ifstream input(graph_file);

    if (input.bad())
        return 1;

    std::shared_ptr<const steiner_graph> graph_ptr(
            new steiner_graph(triangulation_file_io::read<steiner_graph>(input)));
    std::cout << "\r\a\tdone, graph has " << std::setw(12) << graph_ptr->node_count() << " nodes and "
              << std::setw(12) << graph_ptr->edge_count()
              << " edges"
              << "\n\t           with " << std::setw(12) << graph_ptr->base_graph().node_count() << " nodes and "
              << std::setw(12)
              << graph_ptr->base_graph().edge_count() << " edges stored explicitly" << std::endl;

    steiner_routing_t router(graph_ptr);
    std::cout << "\tbtw: expected size per node: " << steiner_routing_t::SIZE_PER_NODE << " and per edge "
              << steiner_routing_t::SIZE_PER_EDGE << std::endl;

    bool done = false;
    while (!done) {
        // get query
        steiner_graph::node_id_type src(0, 0);
        steiner_graph::node_id_type dest(0, 0);
        char mode = 'A';

        if (argc > 6) {
            src.edge = std::stoi(argv[3]);
            src.steiner_index = std::stoi(argv[4]);
            dest.edge = std::stoi(argv[5]);
            dest.steiner_index = std::stoi(argv[6]);
            done = true;
        } else {
            std::cout << "src node (base edge and index): " << std::flush;
            std::cin >> src.edge >> src.steiner_index;
            std::cout << "dest node (base edge and index): " << std::flush;
            std::cin >> dest.edge >> src.steiner_index;
            std::cout << "mode (B = Bidirectional Dijkstra, A = A*) : A" << std::flush;
            //std::cin >> mode;
            std::cout << std::endl;
        }

        // check that query is valid
        if (src.edge < 0 || dest.edge < 0 || src.edge >= graph_ptr->edge_count() ||
            dest.edge >= graph_ptr->node_count())
            break;

        // setup writer for graphs to show
        std::string target_directory = std::format("{}/{}_{}_{}", output_directory, mode, (int) src.edge,
                                                   (int) dest.edge);
        std::filesystem::create_directory(target_directory);
        std::string beeline_file = std::format("{}/beeline.gl", target_directory);
        std::string route_file = std::format("{}/route.gl", target_directory);
        std::string tree_file = std::format("{}/tree.gl", target_directory);
        std::string info_file = std::format("{}/info.gl", target_directory);
        std::ofstream output_beeline(beeline_file);
        std::ofstream output_route(route_file);
        std::ofstream output_tree(tree_file);
        std::ofstream output_info(info_file);


        Query<steiner_graph> query{src, dest};
        Result<steiner_graph> result;
        result = perform_query(*graph_ptr, router, query);

        // make graph from route to display
        auto route_subgraph = graph_ptr->make_subgraph(result.route);
        auto route_graph = std_graph_t::make_graph(*graph_ptr, route_subgraph);

        // make graph from the shortest path trees
        auto tree_graph = std_graph_t::make_graph(*graph_ptr, result.trees);

        // make beeline
        std::vector<node_t> nodes = {graph_ptr->node(src), graph_ptr->node(dest)};
        unidirectional_adjacency_list<int, edge_t>::adjacency_list_builder edges(2);
        edges.add_edge(0, 1, {0});
        std_graph_t beeline = std_graph_t::make_graph(std::move(nodes),
                                                      adjacency_list<int, edge_t>::make_bidirectional(edges.get()));

        // write output graphs
        gl_file_io::write(output_tree, tree_graph, 3, 4);
        gl_file_io::write(output_route, route_graph, 12, 5);
        gl_file_io::write(output_beeline, beeline, 12, 5);

        // print stats about route computation
        output_info << "path: " << result.route << '\n';
        output_info << "path has cost: " << graph_ptr->path_length(result.route) << '\n';
        output_info << "searches visited " << tree_graph.node_count() << " nodes";
        output_info << "and took " << result.duration << '\n';

        std::cout << "\tpath: " << result.route << '\n';
        std::cout << "\tpath has cost: " << graph_ptr->path_length(result.route) << '\n';
        std::cout << "\tsearches visited " << tree_graph.node_count() << " nodes";
        std::cout << "\tand took " << result.duration << '\n';

        output_route.close();
        output_tree.close();
        output_info.close();
    }

    return 0;
}
