#pragma once

#include "../query.h"
#include "../routing.h"
#include <array>
#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>

#define assert_equal(actual, expected)                                                                                 \
  {                                                                                                                    \
    if (expected != actual)                                                                                            \
    {                                                                                                                  \
      std::cout << "expected " << expected << ", but got " << actual << std::endl;                                     \
      assert (expected == actual);                                                                                     \
    }                                                                                                                  \
  }

std::vector<Query>
get_queries(std::ifstream &input) {
    std::vector<Query> result;
    stream_encoders::encode_text f;
    while (input) {
        auto s = f.read<node_id_t>(input);
        auto t = f.read<node_id_t>(input);
        result.push_back({s, t});
    }

    return result;
}

std::vector<distance_t>
get_distances(std::ifstream &input) {
    std::vector<distance_t> result;
    stream_encoders::encode_text f;
    while (input) {
        auto d = f.read<distance_t>(input);
        result.push_back(d);
    }

    return result;
}

template<typename Graph, typename Router>
void
test_routing(const Graph &graph, Router &router, const std::vector<Query> &queries,
             const std::vector<distance_t> &expected) {
    for (int i = 0; i < queries.size(); i++) {
        node_id_t from = queries[i].from;
        node_id_t to = queries[i].to;

        std::cout << "routing from " << from << " to " << to << ": " << std::flush;

        router.init(from, to);
        router.compute_route();
        if (router.route_found()) {
            path route = router.route();

            std::cout << "path found " << route << std::endl;

            assert_equal (expected[i], router.distance());
        } else
            std::cout << "no path found " << std::endl;

        std::cout << "done" << std::endl;
    }
};

template<typename Graph, typename Router1, typename Router2>
void
compare_routing(const Graph &graph, Router1 &router1, Router2 &router2, const std::vector<Query> &queries) {
    for (int i = 0; i < queries.size(); i++) {
        auto res1 = perform_query(graph, router1, queries[i]);
        auto res2 = perform_query(graph, router2, queries[i]);

        output info;
        info.query = false;
        info.distance = true;
        info.timing = true;

        compare_results(res1, res2, info);
    }
}

template<typename Graph, typename Router>
void
test_routing(const Graph &graph, Router &router, const std::vector<Query> &queries) {
    for (int i = 0; i < queries.size(); i++) {
        auto res = perform_query(graph, router, queries[i]);
        output info;
        info.query = false;
        info.distance = true;
        info.timing = true;
        print_result(res, info);
    }
}

template<typename node_id, typename edge>
void
assert_adjacency_list_equal(const adjacency_list< node_id, edge> &list, size_t expected_node_count,
                            size_t expected_edge_count, std::vector<int> &expected_offsets,
                            std::vector<adjacency_list_edge<node_id, edge>> &expected_edges);

// TODO return bool
template<typename node_id, typename edge>
void
assert_adjacency_list_equal(const adjacency_list<node_id, edge> &list, size_t expected_node_count,
                            size_t expected_edge_count, std::vector<int> &expected_offsets,
                            std::vector<adjacency_list_edge<node_id, edge>> &expected_edges) {
//    assert_equal (list.node_count(), expected_node_count);
//    assert_equal (list.edge_count(), expected_edge_count);
//
//    for (size_t node_index = 0; node_index < expected_offsets.size(); node_index++) {
//        assert_equal (list._M_offsets(node_index), expected_offsets[node_index]);
//    }
//
//    for (size_t edge_index = 0; edge_index < expected_edges.size(); edge_index++) {
//        assert_equal (list.destination(edge_index), expected_edges[edge_index].destination);
//        assert_equal (list.source(edge_index), expected_edges[edge_index].source);
//        assert_equal (list.edge(edge_index).cost, expected_edges[edge_index].info.cost);
//    }
}

template<typename N, typename E>
void
test_routes(router<N, E> *router, const std::vector<node_id_t> &sources, const std::vector<node_id_t> &targets,
            const std::vector<distance_t> &distances);

template<typename N, typename E>
void
test_routes(router<N, E> *router, const std::vector<node_id_t> &sources, const std::vector<node_id_t> &targets,
            const std::vector<distance_t> &distances) {
    for (int i = 0; i < sources.size(); i++) {
        node_id_t from = sources[i];
        node_id_t to = targets[i];
        router->compute_route(from, to);
        assert_equal (distances[i], router->distance());
    }
}

template<typename Reader, typename G, typename Check>
bool
test_read(const std::string &filename, Check check) {
    std::cout << "reading from " << filename << ": " << std::endl;
    std::ifstream input;
    input.open(filename);

    if (input.bad() || input.eof())
        std::cout << " bad file" << std::endl;

    Reader reader;
    G graph(reader.template read<G>(input));

    bool success = check(graph);

    if (success)
        std::cout << " successful" << std::endl;
    return success;
}
