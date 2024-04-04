#include "Client.h"

#include "Graph_impl.h"
#include "Router_impl.h"
#include "Query_impl.h"

#include "../routing_impl.h"
#include "../util/memory_usage_impl.h"
#include "../util/csv_impl.h"

#include <thread>


template<typename GraphT>
Client::Client(GraphT &&__graph)
        : _graph{std::forward<GraphT>(__graph)}, statistics{COLUMNS} {
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
        : _graph{std::forward<GraphT>(graph)}, _router{_graph, std::forward<RoutingT>(router)}, statistics{COLUMNS} {
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
    std::cout << "reading graph from " << path << "...";

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

    std::cout << "\b\b\b, done\n";

    _graph.write_graph_stats(statistics);
    statistics.put(Statistics::MEMORY_USAGE_GRAPH, (vm_graph - vm) / 1024);
};

template<>
void Client::read_graph_file(std::string path, double epsilon) {
    std::cout << "reading graph from " << path << "...";
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

    std::cout << "\b\b\b, done\n";

    _graph.write_graph_stats(statistics);
    statistics.put(Statistics::MEMORY_USAGE_GRAPH, (vm_graph - vm) / 1024);
};

void Client::write_info(std::ostream& output) const {
    if (_result.route_found()) {
    output << "\acost:                                     " << statistics.get(Statistics::COST);
    output << "\npath:                                     some long path"; // TODO print path
    } else {
    output << "\acost:                                     inf";
    output << "\npath:                                     not found";
    }

    output << "\ntime:                                     " << statistics.get(TIME)
           << "\nsearch stats:                             "
           << "\n    nodes visited:                        " << statistics.get(TREE_SIZE)
           << "\n    times pulled (num of nodes labelled): " << statistics.get(QUEUE_PULL_COUNT)
           << "\n    times pushed (num of edges relaxed):  " << statistics.get(QUEUE_PUSH_COUNT)
           << "\n    edges checked (i.e. cost computed):   " << statistics.get(EDGES_CHECKED)
           << "\ntree pruning stats:                       "
           << "\n    vertices visited:                     " << statistics.get(NEIGHBORS_BASE_NODE_COUNT)
           << "\n    vertex neighbors:                     " << statistics.get(NEIGHBORS_BASE_NODE_NEIGHBORS_COUNT)
           << "\n    boundary vertices visited:            " << statistics.get(NEIGHBORS_BOUNDARY_NODE_COUNT)
           << "\n    boundary vertex neighbors:            " << statistics.get(NEIGHBORS_BOUNDARY_NODE_NEIGHBORS_COUNT)
           << "\n    steiner points visited:               " << statistics.get(NEIGHBORS_STEINER_POINT_COUNT)
           << "\n    steiner point neighbors:              " << statistics.get(NEIGHBORS_STEINER_POINT_NEIGHBORS_COUNT)
           << "\n    steiner point search iterations:      " << statistics.get(NEIGHBORS_STEINER_POINT_ANGLE_CHECK_COUNT);

    output <<"\n\n";
}

void Client::write_tree_file(std::string path) const {
    std::string prefix(path.substr(0, path.find_last_of('/') + 1));
    std::string filename = path.substr(path.find_last_of('/'));
    std::string name = filename.substr(0, filename.find_first_of('.'));
    std::string suffix = filename.substr(filename.find_first_of('.'), filename.size());

    std::string fwd{prefix + name + suffix};
    _result.tree_forward().write_graph_file(fwd, tree_color, 1);
}

void Client::compute_route(long from, long to) {
    ensure_router();
    _query = {};
    _result = {};

    _router.compute_route(from, to);

    // store memory usage
    double vm, res;
    process_mem_usage(vm, res);
    statistics.put(Statistics::MEMORY_USAGE_FINAL, vm / 1024);

    _query = _router.query();
    _result = _router.result();
    _query.write(statistics);
    _result.write(statistics);
}

void Client::compute_one_to_all(long from) {
    ensure_router();
    _query = {};
    _result = {};

    _router.compute_route(from, -1);

    // store memory usage
    double vm, res;
    process_mem_usage(vm, res);
    statistics.put(Statistics::MEMORY_USAGE_FINAL, vm / 1024);

    _query = _router.query();
    _result = _router.result();
    _query.write(statistics);
    _result.write(statistics);
}

void Client::compute_one_to_all(long from, std::ostream &out) {
    ensure_router();
    _query = {};
    _result = {};

    _router.compute_route(from, -1, out);

    // store memory usage
    double vm, res;
    process_mem_usage(vm, res);
    statistics.put(Statistics::MEMORY_USAGE_FINAL, vm / 1024);

    _query = _router.query();
    _result = _router.result();
    _query.write(statistics);
    _result.write(statistics);
}

void Client::ensure_router() {
    if (!_router) {
        _router = {_graph, _routing_config};
    }
}
