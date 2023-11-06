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

#include <chrono>

int
main(int argc, char const *argv[]) {
    // std::string directory = std::string (argv[1]) + "/aegaeis/";
    // std::string graph_filename = "pruned.graph";

    std::string graph_file;
    std::cout << "graph file: ";
    std::cin >> graph_file;

    std::string output_directory;
    std::cout << "output directory: ";
    std::cin >> output_directory;

    // read triangulation
    std::cout << "reading graph from " << graph_file << "... " << std::flush;
    std::ifstream input(graph_file);

    if (input.bad())
        return 1;

    std::shared_ptr<const steiner_graph> graph_ptr(new steiner_graph(triangulation_file_io::read_steiner(input)));
    std::cout << "done, graph has " << graph_ptr->node_count() << " nodes and " << graph_ptr->edge_count() << " edges"
              << std::endl;

    steiner_routing_t router(graph_ptr);

    while (true) {
        // get query
        steiner_graph::node_id_type src(0, 0), dest(0, 1);
        char mode;
        std::cout << "src node: " << std::flush;
        std::cin >> src.edge;
        std::cout << "dest node: " << std::flush;
        std::cin >> dest.edge;
        std::cout << "mode (B = Bidirectional Dijkstra, A = A*) : " << std::flush;
        std::cin >> mode;

        // check that query is valid
        if (src.edge < 0 || dest.edge < 0 || src >= graph_ptr->node_count() || dest >= graph_ptr->node_count())
            break;

        // setup writer for graphs to show
        std::string route_file = std::format("{}/route_{}_{}_{}.gl", output_directory, mode, (int) src.edge, (int) dest.edge);
        std::string tree_file = std::format("{}/tree_{}_{}_{}.gl", output_directory, mode, (int) src.edge, (int) dest.edge);
        std::string info_file = std::format("{}/info_{}_{}_{}.gl", output_directory, mode, (int) src.edge, (int) dest.edge);
        std::ofstream output_route(route_file);
        std::ofstream output_tree(tree_file);
        std::ofstream output_info(info_file);
        gl_file_io writer;

        //
        steiner_graph::path route;
        steiner_graph::subgraph tree_subgraph;
        std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> before;
        std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> after;

        switch (mode) {
            case 'A' ... 'B':
                router.init(src, dest);
                before = std::chrono::high_resolution_clock::now();
                router.compute_route();
                after = std::chrono::high_resolution_clock::now();

                tree_subgraph = router.shortest_path_tree();
                if (router.route_found()) {
                    route = router.route();
                }
                break;

            default:
                break;
        }

        // get time
        std::chrono::duration<double, std::milli> routing_time = after - before;

        // make graph from route to display
        auto route_subgraph = graph_ptr->make_subgraph(route);
        auto route_graph = std_graph_t::make_graph(*graph_ptr, route_subgraph);

        // make graph from the shortest path trees
        auto tree_graph = std_graph_t::make_graph(*graph_ptr, tree_subgraph);

        // write output graphs
        writer.write(output_tree, tree_graph, 3, 4);
        writer.write(output_route, route_graph, 12, 5);

        // print stats about route computation
        output_info << "path: " << route << '\n';
        output_info << "path has cost: " << graph_ptr->path_length(route) << '\n';
        output_info << "searches visited " << tree_graph.node_count() << " nodes";
        output_info << "and took " << routing_time << '\n';

        std::cout << "\tpath: " << route << '\n';
        std::cout << "\tpath has cost: " << graph_ptr->path_length(route) << '\n';
        std::cout << "\tsearches visited " << tree_graph.node_count() << " nodes";
        std::cout << "\tand took " << routing_time << '\n';

        output_route.close();
        output_tree.close();
        output_info.close();
    }

    return 0;
}