#pragma once

#include "Query.h"
#include "Graph.h"
#include "Router.h"

#include "../triangulation/steiner_graph.h"
#include "../file-io/triangulation_file_io.h"
#include "../util/csv.h"

#include <fstream>
#include <memory>

class Client {
public:
private:
    Graph _graph;
    Router _router;

    Query _query;
    Result _result;

    table statistics;

public:
    Client() : _graph(), _router(), _query(), _result(), statistics(COLUMNS) {statistics.new_line();};

    template<typename GraphT>
    Client(GraphT &&graph);

    template<typename GraphT, typename RouterT>
    Client(GraphT &&graph, RouterT &&router);

    template<typename ...Args>
    void read_graph_file(std::string path, Args... args);

    void write_graph_file(std::string path) { _graph.write_graph_file(path); }

    void compute_route(int from, int to) {
        _router.compute_route(from, to);
        _query = _router.query();
        _result = _router.result();
        _query.write(statistics);
        _result.write(statistics);
    };

    Result &result() { return _result; }

    Query &query() { return _query; }

    void compute_one_to_all(int from) {  /*_router.compute_one_to_all(from);*/ };

    void compute_one_to_all(int from, std::ostream &out) { /*_router.compute_one_to_all(from, out);*/ };

    void write_route_file(std::string path) const { _result.path().write_graph_file(path); };

    void write_tree_file(std::string path) const {
        std::string prefix(path.substr(0, path.find_last_of('/') + 1));
        std::string filename = path.substr(path.find_last_of('/'));
        std::string name = filename.substr(0, filename.find_first_of('.'));
        std::string suffix = filename.substr(filename.find_first_of('.'), filename.size());

        std::string fwd{prefix + name + "_forward" + suffix};
        std::string bwd{prefix + name + "_backward" + suffix};
        if (_result.tree_forward().node_count() < 1000000)
            _result.tree_forward().write_graph_file(fwd);
        if (_result.tree_backward().node_count() < 1000000)
            _result.tree_backward().write_graph_file(bwd);
    };

    void write_beeline_file(std::string path) const { _query.beeline().write_graph_file(path); };

    void write_csv(std::ostream &output) const { format_csv(statistics, output); };

    void write_csv_header(std::ostream &output) const { format_header(statistics, output); };

    void write_info(std::ostream &output) const {
        output <<   "time:                                 " << _result.duration()
               << "\nnodes visited:                        " << _result.tree_forward().node_count() << " + "
               << _result.tree_backward().node_count()
               << "\ntimes pulled (num of nodes labelled): " << _result.pull_count()
               << "\ntimes pushed (num of edges relaxed):  " << _result.push_count();
        if (_result.route_found()) {
            output << "\npath:                                 "; // TODO print path
            output << "\ncost:                                 " << _result.distance() << '\n';
        }
        output << std::flush;
    };

    void write_graph_stats(std::ostream &output) const { _graph.write_graph_stats(output); };

    void write_subgraph_file(std::string &path, coordinate_t bottom_left, coordinate_t top_right) const {
        _graph.write_subgraph_file(path, bottom_left, top_right);
    };

    void write_query(std::ostream &output) const { _query.write(output); }

    int node_count() const { return _graph.node_count(); }

    int edge_count() const { return _graph.edge_count(); }
};

