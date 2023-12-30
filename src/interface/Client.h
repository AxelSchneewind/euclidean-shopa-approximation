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
    Client() = default;

    template<typename GraphT>
    Client(GraphT&& graph);

    template<typename GraphT, typename RouterT>
    Client(GraphT&& graph, RouterT&& router);

    template<typename ...Args>
    void read_graph_file(std::string path, Args... args);

    void write_graph_file(std::string path) { _graph.write_graph_file(path); }

    void compute_route(int from, int to) {
        _router.compute_route(from, to);
        _query = _router.query();
        _result = _router.result();
    };

    Result& result() {return _result;}

    void compute_one_to_all(int from) {  /*_router.compute_one_to_all(from);*/ };
    void compute_one_to_all(int from, std::ostream& out) { /*_router.compute_one_to_all(from, out);*/ };

    void write_route_file(std::string path) const { _result.path().write_graph_file(path); };

    void write_tree_file(std::string path) const { _result.tree_forward().write_graph_file(path); };

    void write_beeline_file(std::string path) const { /*_result.beeline().write_graph_file_gl(output);*/ };

    void write_csv(std::ostream &output) const { format_csv(statistics, output); };
    void write_csv_header(std::ostream &output) const { format_header(statistics, output); };

    void write_info(std::ostream &output) const {
     if (_result.route_found()) {
         output << "path: "; // TODO print path
         output << '\n'
                << "has cost " << _result.distance() << ","
                << " with beeline distance "
                << _query.beeline_distance() << ", search visited "
                << _result.tree_forward().node_count() << " + " << _result.tree_backward().node_count() << " nodes and took "
                << _result.duration() << std::endl;
     }
    };

    void write_graph_stats(std::ostream &output) const { _graph.write_graph_stats(output); };

    void write_subgraph_file(std::string &path, coordinate_t bottom_left, coordinate_t top_right) const {
        _graph.write_subgraph_file(path, bottom_left, top_right);
    };

    void write_query(std::ostream &output) const { _query.write(output); }

    int node_count() const { return _graph.node_count(); }

    int edge_count() const { return _graph.edge_count(); }
};

