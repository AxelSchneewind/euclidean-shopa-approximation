#include <format>
#include <fstream>

#include "file-io/fmi_file_io.h"
#include "file-io/gl_file_io.h"
#include "file-io/triangulation_file_io.h"
#include "graph/graph.h"

#include "routing.h"

#include <chrono>


ch_graph_t::subgraph_type
unpack(const ch_graph_t &graph, const ch_graph_t::subgraph_type &subgraph) {
    std::vector<node_id_t> nodes;
    std::vector<edge_id_t> edges;

    std::queue<edge_id_t> remaining;
    for (auto shortcut: subgraph.edges) {
        nodes.push_back(graph.topology().source(shortcut));
        nodes.push_back(graph.topology().destination(shortcut));
        remaining.push(shortcut);

        while (!remaining.empty() && remaining.size() < 1000) {
            edge_id_t edge = remaining.front();
            remaining.pop();

            // take halves of shortcut
            auto a = graph.topology().edge(edge).edgeA;
            auto b = graph.topology().edge(edge).edgeB;
            if (!is_none(a) && !is_none(b)) {
                remaining.push(a);
                nodes.push_back(graph.topology().destination(a));
                remaining.push(b);
            } else {
                edges.push_back(edge);
            }
        }
    }

    // TODO
    // nodes = remove_duplicates(std::move(nodes));

    return {std::move(nodes), std::move(edges)};
}


int
main(int argc, char const *argv[]) {
    if (argc < 1)
        return 1;

    std::string graph_file;
    std::cout << "graph file: ";
    std::cin >> graph_file;

    // read graph
    std::cout << "reading graph from " << graph_file << ":";
    std::ifstream input(graph_file);

    if (input.bad())
        return 1;

    std::shared_ptr<const ch_graph_t> graph_ptr(new ch_graph_t(fmi_file_io::read<ch_graph_t>(input)));
    ch_routing_t router(graph_ptr);

    std::cout << "done" << std::endl;

    std::string output_directory;
    std::cout << "output directory: ";
    std::cin >> output_directory;

    while (true) {
        // get query
        node_id_t src, dest;
        char mode;
        std::cout << "src node: " << std::flush;
        std::cin >> src;
        std::cout << "dest node: " << std::flush;
        std::cin >> dest;
        std::cout << "mode (B = Bidirectional Dijkstra, A = A*) : " << std::flush;
        std::cin >> mode;

        // check that query is valid
        if (src < 0 || dest < 0 || src >= graph_ptr->node_count() || dest >= graph_ptr->node_count())
            break;

        // setup writer for graphs to show
        std::string shortcuts_file = std::format("{}/shortcuts_{}_{}_{}.gl", output_directory, mode, src, dest);
        std::string route_file = std::format("{}/route_{}_{}_{}.gl", output_directory, mode, src, dest);
        std::string tree_file = std::format("{}/tree_{}_{}_{}.gl", output_directory, mode, src, dest);
        std::string info_file = std::format("{}/info_{}_{}_{}.gl", output_directory, mode, src, dest);
        std::ofstream output_shortcuts(shortcuts_file);
        std::ofstream output_route(route_file);
        std::ofstream output_tree(tree_file);
        std::ofstream output_info(info_file);
        gl_file_io writer;

        //
        ch_graph_t::path_type route;
        ch_graph_t::subgraph_type tree_subgraph;
        std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> before;
        std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> after;

        switch (mode) {
            case 'A' ... 'B':
                before = std::chrono::high_resolution_clock::now();
                router.init(src, dest);
                router.compute_route();
                after = std::chrono::high_resolution_clock::now();

                if (router.route_found()) {
                    route = router.route();
                    tree_subgraph = router.shortest_path_tree();
                }
                break;
            default:
                break;
        }

        std::cout << "routing done" << std::endl;

        // get time
        std::chrono::duration<double, std::milli> routing_time = after - before;

        // make graph from route to display
        auto shortcuts_subgraph = graph_ptr->make_subgraph(route);
        auto shortcuts_graph = std_graph_t::make_graph(*graph_ptr, shortcuts_subgraph);
        auto route_subgraph = unpack(*graph_ptr, shortcuts_subgraph);
        auto route_graph = std_graph_t::make_graph(*graph_ptr, route_subgraph);

        // make graph from shortest path tree
        auto tree_graph = std_graph_t::make_graph(*graph_ptr, tree_subgraph);

        // write output graphs
        if (route_graph.node_count() > 0) {
            writer.write(output_shortcuts, shortcuts_graph, 10, 6);
            writer.write(output_route, route_graph, 6, 3);
            writer.write(output_tree, tree_graph, 3, 4);
        }

        // print stats about route computation
        output_info << "path: " << route << '\n';
        output_info << "path has cost: " << graph_ptr->path_length(route) << '\n';
        output_info << "searches visited " << tree_graph.node_count() << " nodes ";
        output_info << "and took " << routing_time << '\n';

        std::cout << "\tpath: " << route << '\n';
        std::cout << "\tpath has cost: " << graph_ptr->path_length(route) << '\n';
        std::cout << "\tsearches visited " << tree_graph.node_count() << " nodes ";
        std::cout << "\tand took " << routing_time << '\n';

        output_shortcuts.close();
        output_route.close();
        output_tree.close();
        output_info.close();
    }

    return 0;
}