#include <thread>
#include "routing.h"
#include "routing_impl.h"

#include "util/memory_usage.h"


template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void Client::ClientModel<GraphT, RoutingT>::write_info(std::ostream &output) const {
    output << "path: " << result->path << '\n';
    auto path_length = graph.path_length(result->path);
    auto beeline_distance = result->beeline_distance;
    output << "path has cost: " << path_length << " with beeline distance "
           << beeline_distance << ", satisfying e >= " << ((path_length / beeline_distance) - 1) << '\n';
    output << "searches visited " << result->trees.node_count() << " nodes ";
    output << "and took " << result->duration << '\n';
}

template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void Client::ClientModel<GraphT, RoutingT>::write_beeline(std::ostream &output) const {
    std::vector<node_t> nodes = {graph.node(query->from), graph.node(query->to)};
    unidirectional_adjacency_list<int, edge_t>::adjacency_list_builder edges(2);
    edges.add_edge(0, 1, {distance(nodes[0].coordinates, nodes[1].coordinates)});
    edges.add_edge(1, 0, {distance(nodes[0].coordinates, nodes[1].coordinates)});
    std_graph_t beeline = std_graph_t::make_graph(std::move(nodes),
                                                  adjacency_list<int, edge_t>::make_bidirectional(edges.get()));

    gl_file_io::write(output, beeline, 2, 5);
}


template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void Client::ClientModel<GraphT, RoutingT>::write_tree_file(std::ostream &output) const {
    auto tree_graph = std_graph_t::make_graph(graph, result->trees);
    gl_file_io::write(output, tree_graph, 1, 4);
}


template<>
void Client::read_graph_file(std::string path) {
    std::ifstream input(path);
    if (path.ends_with(".graph"))
        pimpl = std::make_unique<ClientModel<steiner_graph, steiner_routing_t>>(
                triangulation_file_io::read_steiner(input, 0.5F));
    if (path.ends_with(".fmi"))
        pimpl = std::make_unique<ClientModel<std_graph_t, std_routing_t>>(fmi_file_io::read<std_graph_t>(input));
    if (path.ends_with(".sch"))
        pimpl = std::make_unique<ClientModel<ch_graph_t, ch_routing_t>>(fmi_file_io::read<ch_graph_t>(input));
    else
        throw std::invalid_argument("unrecognized file ending");

// ...

    input.close();
};

template<>
void Client::read_graph_file(std::string path, float epsilon) {
    std::ifstream input(path);
    if (path.ends_with(".graph"))
        pimpl = std::make_unique<ClientModel<steiner_graph, steiner_routing_t>>(
                triangulation_file_io::read_steiner(input, epsilon));
    else
        throw std::invalid_argument("unrecognized file ending");
    // ...

    input.close();
};


template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void Client::ClientModel<GraphT, RoutingT>::write_route_file(std::ostream &output) const {
    auto route_subgraph = graph.make_subgraph(result->path);
    auto route_graph = std_graph_t::make_graph(graph, route_subgraph);
    gl_file_io::write(output, route_graph, 6, 5);
}

template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void Client::ClientModel<GraphT, RoutingT>::compute_one_to_all(int from) {
    query = std::make_unique<Query<GraphT>>(from, -1);
}


template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void Client::ClientModel<GraphT, RoutingT>::compute_one_to_all(int from, std::ostream &output) {
    query = std::make_unique<Query<GraphT>>(from, -1);
    throw std::exception();
}

template<>
void Client::ClientModel<steiner_graph, steiner_routing_t>::compute_one_to_all(int from, std::ostream &output) {
    query = std::make_unique<Query<steiner_graph>>(from, -1);
    // using distance_labels = frontier_labels<node_cost_pair<steiner_graph::node_id_type, steiner_graph::distance_type>>;
    using distance_labels = frontier_labels<node_cost_pair<steiner_graph::node_id_type, steiner_graph::distance_type>, label_type<steiner_graph>>;
    using distance_dijkstra = dijkstra<steiner_graph, dijkstra_queue<steiner_graph, node_cost_pair<steiner_graph::node_id_type, steiner_graph::distance_type>>, use_all_edges<steiner_graph>, distance_labels>;

    std_graph_t::node_id_type last_node = none_value<std_graph_t::node_id_type>;
    distance_dijkstra distances(graph, {graph}, {graph}, {graph, 10.0});
    distances.init(from);

    while (!distances.queue_empty()) [[likely]] {
        distances.step();

        auto ncp = distances.current();
        if (ncp.node.steiner_index == 0) [[unlikely]] {
            steiner_graph::triangle_node_id_type base_node_id = graph.base_graph().source(ncp.node.edge);
            steiner_graph::triangle_node_id_type pred_node_id = graph.base_graph().source(ncp.predecessor.edge);

            if (base_node_id != last_node) {
                output << base_node_id << ',' << ncp.distance << '\n';
                last_node = base_node_id;

                double vm, res;
                process_mem_usage(vm, res);
                std::cout << "\rdistance: " << std::setw(10) << std::setprecision(5) << distances.current().distance
                          << ", node aggregates currently expanded: " << std::setw(10)
                          << distances.labels().aggregate_count()
                          << ", memory usage : VM " << vm / 1024 << "MiB, RES " << res / 1024 << "MiB" << std::flush;
            }
        }
    }

    std::cout << std::endl;
}


template<>
void Client::ClientModel<steiner_graph, steiner_routing_t>::compute_one_to_all(int from) {
    query = std::make_unique<Query<steiner_graph>>(from, -1);
    std::ofstream out(nullptr);
    compute_one_to_all(from, out);
}

template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void Client::ClientModel<GraphT, RoutingT>::compute_route(int from, int to) {
    query = std::make_unique<Query<GraphT>>(from, to);
    // create thread to show progress
    std::thread status([&]() -> void {
        while (is_none(router.mid_node())) {
            std::cout << "\rdistances: " << std::setw(10) << std::setprecision(5)
                      << router.forward_distance() << ", "
                      << std::setw(10) << std::setprecision(5) << router.backward_distance()
                      << " of total < " << result->beeline_distance << std::flush;
            usleep(5000);
        }
        std::cout << "\rdone                                                                  " << std::flush << "\r";
    });

    router.init(query->from, query->to);

    // setup timing
    auto before = std::chrono::high_resolution_clock::now();

    router.compute_route();

    auto after = std::chrono::high_resolution_clock::now();

    status.join();

    auto route = router.route();
    auto tree = router.shortest_path_tree();
    result = std::make_unique<Result<GraphT>>(
            *query,
            router.route_found(),
            graph.path_length(route),
            distance(graph.node(query->from).coordinates, graph.node(query->to).coordinates),
            tree,
            route,
            tree.node_count(),
            after - before);

}

template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void Client::ClientModel<GraphT, RoutingT>::write_graph_file(std::ostream &output) const {
    throw std::exception();
}

template<>
void Client::ClientModel<steiner_graph, steiner_routing_t>::write_graph_file(std::ostream &output) const {
    triangulation_file_io::write(output, graph);
}

template<>
void Client::ClientModel<std_graph_t, std_routing_t>::write_graph_file(std::ostream &output) const {
    fmi_file_io::write(output, graph);
}


template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void
Client::ClientModel<GraphT, RoutingT>::write_graph_stats(std::ostream &output) const {
    output << "\r\a\tdone, graph has "
           << std::setw(12) << graph.node_count() << " nodes and "
           << std::setw(12) << graph.edge_count() << " edges";

    double vm, res;
    process_mem_usage(vm, res);
    output << "Memory usage with graph loaded and routing set up: VM " << vm / 1024 << "MiB, RES " << res / 1024
           << "MiB"
           << std::endl;
}


template<>
void
Client::ClientModel<steiner_graph, steiner_routing_t>::write_graph_stats(std::ostream &output) const {
    output << "\r\a\tdone, graph has "
           << std::setw(12) << graph.node_count() << " nodes and "
           << std::setw(12) << graph.edge_count() << " edges"
           << "\n\t           with "
           << std::setw(12) << graph.base_graph().node_count() << " nodes and "
           << std::setw(12) << graph.base_graph().edge_count() << " edges stored explicitly" << std::endl;

    output << "\tgraph: expected size per node: "
           << std::setw(3) << steiner_graph::SIZE_PER_NODE << " and per edge "
           << std::setw(3) << steiner_graph::SIZE_PER_EDGE << " -> "
           << graph.base_graph().node_count() * steiner_graph::SIZE_PER_NODE / 1024 / 1024 << "MiB" << " + "
           << graph.base_graph().edge_count() * steiner_graph::SIZE_PER_EDGE / 1024 / 1024 << "MiB"
           << std::endl;

    double vm, res;
    process_mem_usage(vm, res);
    output << "\tactual memory usage with graph loaded: VM " << vm / 1024 << "MiB, RES " << res / 1024 << "MiB"
           << std::endl;
}

