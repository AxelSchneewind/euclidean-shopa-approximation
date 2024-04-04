#pragma once

#include "Query.h"
#include "Graph.h"
#include "Router.h"

#include "../triangulation/steiner_graph.h"
#include "../file-io/triangulation_file_io.h"
#include "../util/csv.h"
#include "RoutingConfig.h"
#include "../util/memory_usage.h"

#include <fstream>
#include <memory>

class Client {
public:
private:
    Graph _graph;
    RoutingConfiguration _routing_config;
    Router _router;

    Query _query;
    Result _result;

    table statistics;

    int tree_color = 3;
    int path_color = 5;
    int beeline_color = 1;

    void ensure_router();

public:
    Client() : statistics(COLUMNS) { statistics.new_line(); }

    template<typename GraphT>
    Client(GraphT&&graph);

    template<typename GraphT, typename RouterT>
    Client(GraphT&&graph, RouterT&&router);

    template<typename... Args>
    void read_graph_file(std::string path, Args... args);

    void write_graph_file(std::string path) const { _graph.write_graph_file(path); }

    void configure(RoutingConfiguration const& config) { _routing_config = config; }

    Result& result() { return _result; }

    Query& query() { return _query; }

    void compute_route(long from, long to);

    void compute_one_to_all(long from);

    void compute_one_to_all(long from, std::ostream& out);

    void write_route_file(std::string path) const { _result.path().write_graph_file(path, path_color, 2); }

    void write_tree_file(std::string path) const;

    void write_beeline_file(std::string path) const { _query.beeline().write_graph_file(path, beeline_color, 1); }

    void write_csv(std::ostream&output) const { format_csv(statistics, output); }

    void write_csv_header(std::ostream&output) const { format_header(statistics, output); }

    void write_info(std::ostream&output) const;

    void write_graph_stats(std::ostream&output) const { _graph.write_graph_stats(output); }

    void write_subgraph_file(std::string&path, coordinate_t bottom_left, coordinate_t top_right) const { _graph.write_subgraph_file(path, bottom_left, top_right); }

    void write_query(std::ostream&output) const { _query.write(output); }

    int node_count() const { return _graph.node_count(); }

    int edge_count() const { return _graph.edge_count(); }
};
