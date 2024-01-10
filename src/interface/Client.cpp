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
    } else if (path.ends_with(".fmi")) {
        _graph.read_graph_file(path);
        process_mem_usage(vm_graph, res_graph);
    } else if (path.ends_with(".sch")) {
        _graph.read_graph_file(path);
        process_mem_usage(vm_graph, res_graph);
    } else if (path.ends_with(".gl")) {
        _graph.read_graph_file(path);
        process_mem_usage(vm_graph, res_graph);
    } else {
        throw std::invalid_argument("unrecognized file ending");
    }
    input.close();

    std::cout << "\b\b\b, done";

    _router = {_graph, _routing_config};

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
    } else {
        throw std::invalid_argument("unrecognized file ending");
    }
    input.close();

    std::cout << "\b\b\b, done";

    _router = {_graph, _routing_config};

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

void Client::write_tree_file(std::string path) const {
    std::string prefix(path.substr(0, path.find_last_of('/') + 1));
    std::string filename = path.substr(path.find_last_of('/'));
    std::string name = filename.substr(0, filename.find_first_of('.'));
    std::string suffix = filename.substr(filename.find_first_of('.'), filename.size());

    std::string fwd{prefix + name + suffix};
    if (_result.tree_forward().node_count() <= max_tree_size)
        _result.tree_forward().write_graph_file(fwd, tree_color, 1);
}
