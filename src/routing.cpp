#include "routing.h"
#include "routing_impl.h"

#include "util/memory_usage.h"
#include "util/csv_impl.h"

#include <thread>

enum Statistics {
    NODE_COUNT,
    EDGE_COUNT,
    STORED_NODE_COUNT,
    STORED_EDGE_COUNT,
    EPSILON,
    MEMORY_USAGE_GRAPH,
    MEMORY_USAGE_FINAL,
    FROM,
    TO,
    FROM_INTERNAL,
    TO_INTERNAL,
    COST,
    BEELINE_DISTANCE,
    EPSILON_SATISFIED,
    TREE_SIZE,
    QUEUE_PULL_COUNT,
    QUEUE_PUSH_COUNT,
    QUEUE_MAX_SIZE,
    TIME,
    PATH,
    NUM_COLUMNS
};

const std::array<std::string, NUM_COLUMNS> COLUMNS{
        "NODE COUNT",
        "EDGE COUNT",
        "STORED NODE COUNT",
        "STORED EDGE COUNT",
        "EPSILON",
        "MEMORY USAGE GRAPH",
        "MEMORY USAGE FINAL",
        "FROM",
        "TO",
        "FROM INTERNAL",
        "TO INTERNAL",
        "COST",
        "BEELINE DISTANCE",
        "EPSILON SATISFIED",
        "TREE SIZE",
        "QUEUE PULL COUNT",
        "QUEUE PUSH COUNT",
        "QUEUE MAX SIZE",
        "TIME",
        "PATH"
};


template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
Client::ClientModel<GraphT, RoutingT>::ClientModel(GraphT &&__graph, bool output_csv)
        : graph{std::move(__graph)}, router{graph}, output_csv{output_csv}, statistics{COLUMNS} {
    statistics.new_line();

    if constexpr (requires(GraphT graph) { graph.epsilon(); graph.base_graph(); }) {
        statistics.put(Statistics::EPSILON, graph.epsilon());
        statistics.put(Statistics::STORED_NODE_COUNT, graph.base_graph().node_count());
        statistics.put(Statistics::STORED_EDGE_COUNT, graph.base_graph().edge_count());
    }

    statistics.put(Statistics::NODE_COUNT, graph.node_count());
    statistics.put(Statistics::EDGE_COUNT, graph.edge_count());

    double vm, res;
    process_mem_usage(vm, res);
    statistics.put(Statistics::MEMORY_USAGE_GRAPH, vm / 1024);
}

template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
Client::ClientModel<GraphT, RoutingT>::ClientModel(GraphT &&graph, RoutingT &&router, bool output_csv)
        : graph{std::move(graph)}, router{std::move(router)}, output_csv(output_csv), statistics{COLUMNS} {
    statistics.new_line();

    if constexpr (requires(GraphT graph) { graph.epsilon(); graph.base_graph(); }) {
        statistics.put(Statistics::EPSILON, graph.epsilon());
        statistics.put(Statistics::STORED_NODE_COUNT, graph.base_graph().node_count());
        statistics.put(Statistics::STORED_EDGE_COUNT, graph.base_graph().edge_count());
    }

    statistics.put(Statistics::NODE_COUNT, graph.node_count());
    statistics.put(Statistics::EDGE_COUNT, graph.edge_count());

    double vm, res;
    process_mem_usage(vm, res);
    statistics.put(Statistics::MEMORY_USAGE_GRAPH, vm / 1024);
}


template<>
void Client::read_graph_file(std::string path, bool output_csv) {
    std::ifstream input(path);

    // remove current implementation
    if (pimpl)
        pimpl.reset();

    if (path.ends_with(".graph"))
        pimpl = std::make_unique<ClientModel<steiner_graph, steiner_routing_t>>(
                triangulation_file_io::read_steiner(input, 0.5F), output_csv);
    else if (path.ends_with(".fmi"))
        pimpl = std::make_unique<ClientModel<std_graph_t, std_routing_t>>(fmi_file_io::read<std_graph_t>(input),
                                                                          output_csv);
    else if (path.ends_with(".sch"))
        pimpl = std::make_unique<ClientModel<ch_graph_t, ch_routing_t>>(fmi_file_io::read<ch_graph_t>(input),
                                                                        output_csv);
    else
        throw std::invalid_argument("unrecognized file ending");

// ...

    input.close();
};

template<>
void Client::read_graph_file(std::string path, float epsilon, bool csv) {
    std::ifstream input(path);

    if (path.ends_with(".graph"))
        pimpl = std::make_unique<ClientModel<steiner_graph, steiner_routing_t>>(
                triangulation_file_io::read_steiner(input, epsilon), csv);
    else
        throw std::invalid_argument("unrecognized file ending");
    // ...

    input.close();
};


template<>
void Client::set_graph(steiner_graph &&graph) {
    pimpl = std::make_unique<ClientModel<steiner_graph, steiner_routing_t>>(std::move(graph));
};

template<>
void Client::set_graph(std_graph_t &&graph) {
    pimpl = std::make_unique<ClientModel<std_graph_t, std_routing_t>>(std::move(graph));
};

template<>
void Client::set_graph(ch_graph_t &&graph) {
    pimpl = std::make_unique<ClientModel<ch_graph_t, ch_routing_t>>(std::move(graph));
};


template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
Query<GraphT> Client::ClientModel<GraphT, RoutingT>::make_query(int from, int to) {
    Query<GraphT> query = {from, to};
    statistics.put(Statistics::FROM_INTERNAL, query.from);
    statistics.put(Statistics::TO_INTERNAL, query.to);
    statistics.put(Statistics::FROM, from);
    statistics.put(Statistics::TO, to);
    return query;
}

template<>
Query<steiner_graph> Client::ClientModel<steiner_graph, steiner_routing_t>::make_query(int from, int to) {
    Query<steiner_graph> query = {graph.from_base_node_id(from),
                                  is_none(to) ? none_value<steiner_graph::node_id_type> : graph.from_base_node_id(to)};
    statistics.put(Statistics::FROM_INTERNAL, query.from);
    statistics.put(Statistics::TO_INTERNAL, query.to);
    statistics.put(Statistics::FROM, from);
    statistics.put(Statistics::TO, to);
    return query;
}


template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void Client::ClientModel<GraphT, RoutingT>::compute_one_to_all(int from) {
    query = std::make_unique<Query<GraphT>>(make_query(from, -1));
}


template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void Client::ClientModel<GraphT, RoutingT>::compute_one_to_all(int from, std::ostream &output) {
    query = std::make_unique<Query<GraphT>>(make_query(from, -1));
    throw std::exception();
}

template<>
void Client::ClientModel<steiner_graph, steiner_routing_t>::compute_one_to_all(int from, std::ostream &output) {
    query = std::make_unique<Query<steiner_graph>>(make_query(from));
    using distance_labels = frontier_labels<node_cost_pair<steiner_graph::node_id_type, steiner_graph::distance_type>, label_type<steiner_graph>>;
    using distance_queue = dijkstra_queue<steiner_graph, node_cost_pair<steiner_graph::node_id_type, steiner_graph::distance_type>>;
    using distance_dijkstra = dijkstra<steiner_graph, distance_queue, distance_labels, default_neighbors<steiner_graph>, use_all_edges<steiner_graph>>;
    double max_dist = 0.4;

    std_graph_t::node_id_type last_node = none_value<std_graph_t::node_id_type>;
    distance_dijkstra distances(graph);
    distances.labels().set_frontier_width(0.5);

    distances.init(query->from);

    std::chrono::time_point<std::chrono::high_resolution_clock> before, after;

    std::thread status([&]() -> void {
        double vm, res;
        while (distances.current().distance < max_dist) {
            if (output_csv) {

            } else {
                process_mem_usage(vm, res);
                std::cout << "\rdistance: " << std::setw(10) << std::setprecision(3) << distances.current().distance
                          << ", node aggregates currently expanded (in labels): " << std::setw(10)
                          << distances.labels().aggregate_count();
                std::cout << ", memory usage : VM " << std::setw(9) << std::setprecision(2) << vm / 1024 << "MiB, RES "
                          << std::setw(9) << std::setprecision(2) << res / 1024 << std::flush;
            }

            usleep(50000);
        }
    });

    before = std::chrono::high_resolution_clock::now();
    while (distances.current().distance < max_dist) [[likely]] {
        distances.step();

        auto ncp = distances.current();
        if (ncp.node.steiner_index == 0) [[unlikely]] {
            steiner_graph::triangle_node_id_type base_node_id = graph.base_graph().source(ncp.node.edge);
            steiner_graph::triangle_node_id_type pred_node_id = graph.base_graph().source(ncp.predecessor.edge);

            if (base_node_id != last_node) {
                output << base_node_id << ',' << ncp.distance << '\n';
                last_node = base_node_id;
            }
        }
    }
    after = std::chrono::high_resolution_clock::now();


    steiner_graph::path_type path {graph, std::vector<steiner_graph::node_id_type>()};
    auto tree = distances.shortest_path_tree();
    result = std::make_unique<Result<steiner_graph>>(*query, router.route_found(), max_dist, max_dist, tree,
                                                     path, tree.node_count(), after - before);


    statistics.put(Statistics::QUEUE_PULL_COUNT, distances.queue().pull_count());
    statistics.put(Statistics::QUEUE_PUSH_COUNT, distances.queue().push_count());
    statistics.put(Statistics::QUEUE_MAX_SIZE, distances.queue().max_size());

    if (!output_csv) {
        std::cout << "\nqueue was pulled from "
                  << distances.queue().pull_count() << " times, pushed to "
                  << distances.queue().push_count() << " times, and had a maximum size of "
                  << distances.queue().max_size() << " elements" << std::endl;
    }

    status.join();
}

template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void Client::ClientModel<GraphT, RoutingT>::write_query(std::ostream &output) const {
    typename GraphT::node_id_type from = query->from;
    typename GraphT::node_id_type to = query->to;
    if (!output_csv)
        output << "from " << from << ", to " << to << std::endl;
}


template<>
void Client::ClientModel<steiner_graph, steiner_routing_t>::compute_one_to_all(int from) {
    query = std::make_unique<Query<steiner_graph>>(make_query(from));
    std::ofstream out(nullptr);
    compute_one_to_all(from, out);
}

template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void Client::ClientModel<GraphT, RoutingT>::compute_route(int from, int to) {
    query = std::make_unique<Query<GraphT>>(make_query(from, to));
    result.reset();

    auto beeline = distance(graph.node(query->from).coordinates, graph.node(query->to).coordinates);

    // create thread to show progress
    bool done = false;
    std::thread status([&]() -> void {
        while (!done) {
            if (output_csv) {
                usleep(1000000);
            } else {
                double vm, res;
                process_mem_usage(vm, res);
                std::cout << "\rdistances: "
                          << std::setw(12) /*<< std::setprecision(3)*/ << router.forward_distance()
                          << " (" << std::setw(12) /*<< std::setprecision(3)*/
                          << router.forward_search().current().value() << "), "
                          << std::setw(12) /*<< std::setprecision(3)*/ << router.backward_distance()
                          << " (" << std::setw(12) /*<< std::setprecision(3)*/
                          << router.backward_search().current().value() << "), "
                          << " of total > " << beeline;

                if constexpr (requires(RoutingT::labels_type && l) { l.aggregate_count(); }) {
                    std::cout << ", node aggregates currently expanded: " << std::setw(10)
                              << router.forward_labels().aggregate_count() + router.backward_labels().aggregate_count();
                }

                std::cout << ", memory usage : VM " << std::setw(9) << vm / 1024 << "MiB, RES " << std::setw(9)
                          << res / 1024
                          << "MiB" << std::flush;
                usleep(100000);
            }
        }

        if (!output_csv) {
            std::cout << "\nqueue was pulled from "
                      << router.forward_search().queue().pull_count() << " times, pushed to "
                      << router.forward_search().queue().push_count() << " times, and had a maximum size of "
                      << router.forward_search().queue().max_size() << " elements" << std::endl;
        }
    });

    router.init(query->from, query->to);

    // setup timing
    auto before = std::chrono::high_resolution_clock::now();

    router.compute_route();

    auto after = std::chrono::high_resolution_clock::now();

    done = true;
    status.join();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>((after - before));
    statistics.put(Statistics::TIME, duration);
    statistics.put(Statistics::BEELINE_DISTANCE, beeline);

    statistics.put(Statistics::QUEUE_PULL_COUNT, router.forward_search().queue().pull_count());
    statistics.put(Statistics::QUEUE_PUSH_COUNT, router.forward_search().queue().push_count());
    statistics.put(Statistics::QUEUE_MAX_SIZE, router.forward_search().queue().max_size());

    double vm, res;
    process_mem_usage(vm, res);
    statistics.put(Statistics::MEMORY_USAGE_FINAL, vm / 1024);

    if (router.route_found()) {
        auto route = router.route();

        using subtree = decltype(router.shortest_path_tree());
        subtree tree{graph};

        if constexpr (requires(GraphT g) { g.epsilon(); }) {
            // if (graph.epsilon() >= 0.1)
            tree = router.shortest_path_tree();
        } else {
            tree = router.shortest_path_tree();
        }

        result = std::make_unique<Result<GraphT>>(*query, router.route_found(), graph.path_length(route), beeline, tree,
                                                  route, tree.node_count(), after - before);

        // store statistics
        if (router.route_found()) {
            auto path_length = graph.path_length(result->path);
            auto beeline_distance = result->beeline_distance;

            statistics.put(Statistics::COST, path_length);
            statistics.put(Statistics::EPSILON_SATISFIED, (path_length / beeline_distance) - 1);
            statistics.put(Statistics::TREE_SIZE, result->trees.node_count());
        }
    }
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
    if (!output_csv) {
        output << "\r\a    graph has "
               << std::setw(12) << graph.node_count() << " nodes and "
               << std::setw(12) << graph.edge_count() / 2 << " edges";

        double vm, res;
        process_mem_usage(vm, res);
        output << "    memory usage with graph loaded and routing set up: VM " << vm / 1024 << "MiB, RES " << res / 1024
               << "MiB"
               << std::endl;
    }
}


template<>
void
Client::ClientModel<steiner_graph, steiner_routing_t>::write_graph_stats(std::ostream &output) const {
    if (!output_csv) {
        output << "\r\a    done, graph has "
               << std::setw(12) << graph.node_count() << " nodes and "
               << std::setw(12) << graph.edge_count() / 2 << " edges"
               << "\n                 with "
               << std::setw(12) << graph.base_graph().node_count() << " nodes and "
               << std::setw(12) << graph.base_graph().edge_count() / 2 << " edges stored explicitly (ε = "
               << graph.epsilon() << ")" << std::endl;

        output << "    graph:  expected size per node: "
               << std::setw(5) << steiner_graph::SIZE_PER_NODE << " and per edge "
               << std::setw(5) << steiner_graph::SIZE_PER_EDGE << " -> "
               << graph.base_graph().node_count() * steiner_graph::SIZE_PER_NODE / 1024 / 1024 << "MiB" << " + "
               << graph.base_graph().edge_count() * steiner_graph::SIZE_PER_EDGE / 1024 / 1024 << "MiB"
               << std::endl;

        output << "    router: expected size per node: "
               << std::setw(5) << steiner_routing_t::SIZE_PER_NODE << " and per edge "
               << std::setw(5) << steiner_routing_t::SIZE_PER_EDGE << " -> "
               << graph.base_graph().node_count() * steiner_routing_t::SIZE_PER_NODE / 1024 / 1024 << "MiB" << " + "
               << graph.base_graph().edge_count() * steiner_routing_t::SIZE_PER_EDGE / 1024 / 1024 << "MiB"
               << std::endl;

        double vm, res;
        process_mem_usage(vm, res);
        output << "    actual memory usage with graph loaded: VM " << vm / 1024 << "MiB, RES " << res / 1024 << "MiB"
               << std::endl;
    }
}


template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>void
Client::ClientModel<GraphT, RoutingT>::write_subgraph_file_gl(std::ostream &output, coordinate_t bottom_left,
                                                              coordinate_t top_right) const {
    throw std::runtime_error("write_subgraph_file not implemented");
}

template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>void
Client::ClientModel<GraphT, RoutingT>::write_subgraph_file(std::ostream &output, coordinate_t bottom_left,
                                                           coordinate_t top_right) const {
    throw std::runtime_error("write_subgraph_file not implemented");
}

bool is_in_rectangle(coordinate_t point, coordinate_t bottom_left, coordinate_t top_right) {
    return bottom_left.latitude <= point.latitude && point.latitude <= top_right.latitude
           && bottom_left.longitude <= point.longitude && point.longitude <= top_right.longitude;
}

template<>
void Client::ClientModel<steiner_graph, steiner_routing_t>::write_subgraph_file(std::ostream &output,
                                                                                coordinate_t bottom_left,
                                                                                coordinate_t top_right) const {
    auto &&all_nodes = graph.node_ids();
    std::vector<steiner_graph::node_id_type> nodes;
    std::vector<steiner_graph::edge_id_type> edges;

    for (auto node_id: all_nodes) {
        if (!is_in_rectangle(graph.node(node_id).coordinates, bottom_left, top_right))
            continue;

        nodes.push_back(node_id);

        for (auto edge: graph.outgoing_edges(node_id)) {
            if (!is_in_rectangle(graph.node(edge.destination).coordinates, bottom_left, top_right))
                continue;

            edges.push_back({node_id, edge.destination});
        }
    }

    steiner_graph::subgraph_type subgraph(graph, std::move(nodes), std::move(edges));

    std_graph_t result = std_graph_t::make_graph(graph, subgraph);

    fmi_file_io::write(output, result);
}


template<>
void Client::ClientModel<steiner_graph, steiner_routing_t>::write_subgraph_file_gl(std::ostream &output,
                                                                                   coordinate_t bottom_left,
                                                                                   coordinate_t top_right) const {
    auto &&all_nodes = graph.node_ids();
    std::vector<steiner_graph::node_id_type> nodes;
    std::vector<steiner_graph::edge_id_type> edges;

    for (auto node_id: all_nodes) {
        if (!is_in_rectangle(graph.node(node_id).coordinates, bottom_left, top_right))
            continue;

        nodes.push_back(node_id);

        for (auto edge: graph.outgoing_edges(node_id)) {
            if (!is_in_rectangle(graph.node(edge.destination).coordinates, bottom_left, top_right))
                continue;

            edges.push_back({node_id, edge.destination});
        }
    }

    steiner_graph::subgraph_type subgraph(graph, std::move(nodes), std::move(edges));

    std_graph_t result = std_graph_t::make_graph(graph, subgraph);

    gl_file_io::write(output, result, 1, 1);
}


template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>void
Client::ClientModel<GraphT, RoutingT>::write_csv_header(std::ostream &output) const {
    format_header(statistics, output);
}


template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>void
Client::ClientModel<GraphT, RoutingT>::write_csv(std::ostream &output) const {
    format_csv_line(statistics, output, statistics.row_count() - 1);
}

template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void Client::ClientModel<GraphT, RoutingT>::write_info(std::ostream &output) const {
    if (result) {
        output << "path: " << (result.route_found() && result.route.nodes.size() < 100) result.route ? " some very long path " << '\n'
               << "has cost " << statistics.get(Statistics::COST) << ","
               << " with beeline distance "
               << statistics.get(Statistics::BEELINE_DISTANCE) << ", satisfying epsilon >= "
               << statistics.get(Statistics::EPSILON_SATISFIED) << ", search visited "
               << statistics.get(Statistics::TREE_SIZE) << " nodes and took "
               << statistics.get(Statistics::TIME) << std::endl;
    }
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
    if (!result)
        return;

    auto tree_graph = std_graph_t::make_graph(graph, result->trees);
    gl_file_io::write(output, tree_graph, 1, 4);
}


template<typename GraphT, typename RoutingT>
requires std::convertible_to<typename RoutingT::graph_type, GraphT>
void Client::ClientModel<GraphT, RoutingT>::write_route_file(std::ostream &output) const {
    if (!result)
        return;

    auto route_subgraph = graph.make_subgraph(result->path);
    auto route_graph = std_graph_t::make_graph(graph, route_subgraph);
    gl_file_io::write(output, route_graph, 2, 5);
}
