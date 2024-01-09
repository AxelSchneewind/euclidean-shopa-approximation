#include "Client.h"

#include "Graph_impl.h"
#include "Router_impl.h"
#include "Query_impl.h"

#include "../routing_impl.h"

#include "../util/memory_usage.h"
#include "../util/csv_impl.h"

#include <thread>


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
void Client::read_graph_file(std::string path) {
    std::cout << "reading graph from " << path << "..." << std::flush;

    std::ifstream input(path);

    double vm, res;
    double vm_graph, res_graph;
    process_mem_usage(vm, res);

    if (path.ends_with(".graph")) {
        _graph.read_graph_file(path, 0.5F);
        process_mem_usage(vm_graph, res_graph);
        _router = {_graph};
    } else if (path.ends_with(".fmi")) {
        _graph.read_graph_file(path);
        process_mem_usage(vm_graph, res_graph);
        _router = {_graph};
    } else if (path.ends_with(".sch")) {
        _graph.read_graph_file(path);
        process_mem_usage(vm_graph, res_graph);
        _router = {_graph};
    } else if (path.ends_with(".gl")) {
        _graph.read_graph_file(path);
        process_mem_usage(vm_graph, res_graph);
        _router = {_graph};
    } else {
        throw std::invalid_argument("unrecognized file ending");
    }
    input.close();

    std::cout << "\b\b\b, done";

    _graph.write_graph_stats(statistics);
    statistics.put(Statistics::MEMORY_USAGE_GRAPH, (vm_graph - vm) / 1024);
};

template<>
void Client::read_graph_file(std::string path, double epsilon) {
    std::cout << "reading graph from " << path << "..." << std::flush;
    std::ifstream input(path);

    double vm, res;
    double vm_graph, res_graph;
    process_mem_usage(vm, res);

    if (path.ends_with(".graph")) {
        _graph.read_graph_file(path, epsilon);
        process_mem_usage(vm_graph, res_graph);
        _router = {_graph};
    } else {
        throw std::invalid_argument("unrecognized file ending");
    }
    input.close();

    std::cout << "\b\b\b, done";

    _graph.write_graph_stats(statistics);
    statistics.put(Statistics::MEMORY_USAGE_GRAPH, (vm_graph - vm) / 1024);
};

void Client::write_info(std::ostream& output) const {
    output << "\atime:                                 " << statistics.get(TIME)
           << "\nnodes visited:                        " << statistics.get(TREE_SIZE)
           << "\ntimes pulled (num of nodes labelled): " << statistics.get(QUEUE_PULL_COUNT)
           << "\ntimes pushed (num of edges relaxed):  " << statistics.get(QUEUE_PUSH_COUNT)
           << "\nedges checked (i.e. cost computed):   " << statistics.get(EDGES_CHECKED);
    if (_result.route_found()) {
    output << "\npath:                                 some long path"; // TODO print path
    output << "\ncost:                                 " << statistics.get(Statistics::COST) << '\n';
    } else {
    output << "\npath:                                 not found";
    output << "\ncost:                                 inf\n";
    }
    output << std::flush;
}
