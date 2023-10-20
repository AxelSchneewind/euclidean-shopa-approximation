#include "all.hpp"
#include <array>
#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>

#define assert_equal(actual, expected)                                                   \
    {                                                                                    \
        if (expected != actual) {                                                        \
            std::cout << "expected " << expected << ", but got " << actual << std::endl; \
            assert(expected == actual);                                                  \
        }                                                                                \
    }

struct Query {
    node_id_t from;
    node_id_t to;
};


std::vector<Query> get_queries(std::ifstream& input){
    std::vector<Query> result;
    while (input) {
        auto s = format_text::read<node_id_t >(input);
        auto t = format_text::read<node_id_t >(input);
        result.push_back({s, t});
    }

    return result;
}


std::vector<distance_t> get_distances(std::ifstream& input){
    std::vector<distance_t> result;
    while (input) {
        auto d = format_text::read<distance_t>(input);
        result.push_back(d);
    }

    return result;
}


template<typename Graph, typename  Router>
void test_routing(const Graph& graph, Router& router, const std::vector<Query>& queries, const std::vector<distance_t>& expected) {
    for(int i = 0; i < queries.size(); i++) {
        node_id_t from = queries[i].from;
        node_id_t to = queries[i].to;

        std::cout << "routing from " << from << " to " << to << ": "  << std::flush;

        router.compute_route(from, to);
        typename Graph::path_t route = router.route();

        std::cout << "path found " << std::endl;

        Graph route_graph = graph.make_graph(graph.make_subgraph(route));

        fmi_format().write(std::cout, route_graph);

        assert_equal(expected[i], router.distance());

        std::cout << "done" << std::endl;
    }
};





template<typename edge>
void
assert_adjacency_list_equal(const adjacency_list<edge> &list, int expected_node_count, int expected_edge_count, std::vector<int> &expected_offsets, std::vector<adjacency_list_edge_t<edge> > &expected_edges);




// TODO return bool
template<typename edge>
void
assert_adjacency_list_equal(const adjacency_list<edge> &list, int expected_node_count, int expected_edge_count, vector<int> &expected_offsets, vector<adjacency_list_edge_t<edge> > &expected_edges) {
    assert_equal(list.node_count(), expected_node_count);
    assert_equal(list.edge_count(), expected_edge_count);

    for (size_t node_index = 0; node_index < expected_offsets.size(); node_index++) {
        assert_equal(list.offset(node_index), expected_offsets[node_index]);
    }

    for (size_t edge_index = 0; edge_index < expected_edges.size(); edge_index++) {
        assert_equal(list.destination(edge_index), expected_edges[edge_index].destination);
        assert_equal(list.source(edge_index), expected_edges[edge_index].source);
        assert_equal(list.edge(edge_index).cost, expected_edges[edge_index].info.cost);
    }
}


template<typename N, typename E>
void
test_routes(routing_t<N, E> *router, const std::vector<node_id_t> &sources, const std::vector<node_id_t> &targets, const std::vector<distance_t> &distances);
template<typename N, typename E>
void
test_routes(routing_t<N, E> *router, const vector<node_id_t> &sources, const vector<node_id_t> &targets, const vector<distance_t> &distances) {
    for (int i = 0; i < sources.size(); i++) {
        node_id_t from = sources[i];
        node_id_t to = targets[i];
        router->compute_route(from, to);
        assert_equal(distances[i], router->distance());
    }
}


template<typename Reader, typename node_t, typename edge_t, typename Check>
bool test_read(const std::string& filename, Check check) {
    std::cout << "reading from " << filename << ": " << std::endl;
    std::ifstream input;
    input.open(filename);

    Reader reader;
    graph_t<node_t, edge_t> graph(reader.template read<node_t, edge_t>(input));

    bool success = check(graph);

    if (success)
        std::cout << " successful" << std::endl;
    return success;
}
