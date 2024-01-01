#include "Client.h"

#include "Graph_impl.h"
#include "Router_impl.h"
#include "Query_impl.h"

#include "../routing_impl.h"

#include "../util/memory_usage.h"
#include "../util/csv_impl.h"

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


template<typename GraphT>
Client::Client(GraphT &&__graph)
        : _graph{std::forward(__graph)}, _router{_graph.get<GraphT const&>()}, statistics{COLUMNS} {
    statistics.new_line();

    if constexpr (requires(GraphT graph) { graph.epsilon(); graph.base_graph(); }) {
        statistics.put(Statistics::EPSILON, __graph.epsilon());
        statistics.put(Statistics::STORED_NODE_COUNT, __graph.base_graph().node_count());
        statistics.put(Statistics::STORED_EDGE_COUNT, __graph.base_graph().edge_count());
    }

    statistics.put(Statistics::NODE_COUNT, __graph.node_count());
    statistics.put(Statistics::EDGE_COUNT, __graph.edge_count());

    double vm, res;
    process_mem_usage(vm, res);
    statistics.put(Statistics::MEMORY_USAGE_GRAPH, vm / 1024);
}

template<typename GraphT, typename RoutingT>
Client::Client(GraphT &&graph, RoutingT &&router)
        : _graph{std::forward(graph)}, _router{_graph, std::forward(router)}, statistics{COLUMNS} {
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

    std::cout << "reading graph from " << path << "..." << std::flush;

    if (path.ends_with(".graph")) {
        _graph.read_graph_file(path, 0.5F);
        _router = {_graph};
    } else if (path.ends_with(".fmi")) {
        _graph.read_graph_file(path);
        _router = {_graph};
    } else if (path.ends_with(".sch")) {
        _graph.read_graph_file(path);
        _router = {_graph};
    } else if (path.ends_with(".gl")) {
        _graph.read_graph_file(path);
        _router = {_graph};
    } else {
        throw std::invalid_argument("unrecognized file ending");
    }

    std::cout << "\b\b\b, done";

// ...

    input.close();
};

template<>
void Client::read_graph_file(std::string path, double epsilon, bool csv) {
    std::ifstream input(path);

    std::cout << "reading graph from " << path << "..." << std::flush;

    if (path.ends_with(".graph")) {
        _graph.read_graph_file(path, epsilon);
        _router = {_graph};
    } else {
        throw std::invalid_argument("unrecognized file ending");
    }
    // ...

    std::cout << "\b\b\b, done";

    input.close();
};

// void Client::compute_one_to_all(int from, std::ostream &output) {
//     query = std::make_unique<Query<steiner_graph>>(make_query(from));
//     // using distance_labels = frontier_labels<node_cost_pair<steiner_graph::node_id_type, steiner_graph::distance_type>, label_type<steiner_graph>>;
//     using distance_labels = steiner_labels<steiner_graph, node_cost_pair<steiner_graph::node_id_type, steiner_graph::distance_type>>;
//     using distance_queue = dijkstra_queue<steiner_graph, node_cost_pair<steiner_graph::node_id_type, steiner_graph::distance_type>>;
//     // using distance_dijkstra = dijkstra<steiner_graph, distance_queue, distance_labels, default_neighbors<steiner_graph>, use_all_edges<steiner_graph>>;
//     using distance_dijkstra = dijkstra<steiner_graph, distance_queue, distance_labels, steiner_neighbors<steiner_graph, distance_labels>, use_all_edges<steiner_graph>>;
//
//     double max_dist = 0.4;
//
//     std_graph_t::node_id_type last_node = none_value<std_graph_t::node_id_type>;
//     distance_dijkstra distances(graph);
//     //distances.labels().set_frontier_width(0.5);
//
//     distances.init(query->from);
//
//     std::chrono::time_point<std::chrono::high_resolution_clock> before, after;
//
//     std::thread status([&]() -> void {
//         double vm, res;
//         while (distances.current().distance < max_dist) {
//             if (output_csv) {
//
//             } else {
//                 process_mem_usage(vm, res);
//                 std::cout << "\rdistance: " << std::setw(10) << std::setprecision(3) << distances.current().distance;
//                           // << ", node aggregates currently expanded (in labels): ";
//                           // << std::setw(10) << distances.labels().aggregate_count();
//                 std::cout << ", memory usage : VM " << std::setw(9) << vm / 1024 << "MiB, RES "
//                           << std::setw(9) << res / 1024 << std::flush;
//             }
//
//             usleep(50000);
//         }
//     });
//
//     before = std::chrono::high_resolution_clock::now();
//     while (distances.current().distance < max_dist) [[likely]] {
//         distances.step();
//
//         auto ncp = distances.current();
//         if (ncp.node.steiner_index == 0) [[unlikely]] {
//             steiner_graph::triangle_node_id_type base_node_id = graph.base_graph().source(ncp.node.edge);
//             steiner_graph::triangle_node_id_type pred_node_id = graph.base_graph().source(ncp.predecessor.edge);
//
//             if (base_node_id != last_node) {
//                 output << base_node_id << ',' << ncp.distance << '\n';
//                 last_node = base_node_id;
//             }
//         }
//     }
//     after = std::chrono::high_resolution_clock::now();
//
//
//     steiner_graph::path_type path {graph, std::vector<steiner_graph::node_id_type>()};
//     auto tree = distances.shortest_path_tree();
//     result = std::make_unique<Result<steiner_graph>>(*query, router.route_found(), max_dist, max_dist, tree,
//                                                      path, tree.node_count(), after - before);
//
//
//     statistics.put(Statistics::QUEUE_PULL_COUNT, distances.queue().pull_count());
//     statistics.put(Statistics::QUEUE_PUSH_COUNT, distances.queue().push_count());
//     statistics.put(Statistics::QUEUE_MAX_SIZE, distances.queue().max_size());
//
//     if (!output_csv) {
//         std::cout << "\nqueue was pulled from "
//                   << distances.queue().pull_count() << " times, pushed to "
//                   << distances.queue().push_count() << " times, and had a maximum size of "
//                   << distances.queue().max_size() << " elements" << std::endl;
//     }
//
//     status.join();
// }

