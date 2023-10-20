#include "../file-io/fmi_file_io_impl.h"
#include "test.h"

static std::vector<Query> QUERIES;

static const std::vector<distance_t> EXPECTED_DISTANCES{
    0, 9, 8, 8, 7, DISTANCE_INF,
    DISTANCE_INF, 0, DISTANCE_INF, DISTANCE_INF, DISTANCE_INF, DISTANCE_INF,
    6, 5, 0, 5, 4, DISTANCE_INF,
    9, 8, 3, 0, 7, DISTANCE_INF,
    10, 2, 4, 1, 0, DISTANCE_INF,
    DISTANCE_INF, DISTANCE_INF, DISTANCE_INF, DISTANCE_INF, DISTANCE_INF, 0
};

void
test_route_toy_fmi(std::string __path) {
    std::string filename = __path + "/toy.fmi";
    std::ifstream input;
    input.open(filename);
    fmi_file_io reader;
    std_graph_t graph(reader.read<node_t, edge_t>(input));

    std_routing_t router(&graph);

  	test_routing (graph, router, QUERIES, EXPECTED_DISTANCES);

    std::cout << "routing in " << filename << " successful" << std::endl;
    return;
};

void
test_route_toy_ch(std::string path) {
    std::string filename = path + "/toy.sch";
    std::ifstream input;
    input.open(filename);
    fmi_file_io reader;
    ch_graph_t graph(reader.read<ch_node_t, ch_edge_t>(input));
    ch_routing_t router(&graph);

    test_routing (graph, router, QUERIES, EXPECTED_DISTANCES);

    std::cout << "routing in " << filename << " successful" << std::endl;
    return;
};

int
main(int argc, char const *argv[]) {
    int node_count = std::sqrt(EXPECTED_DISTANCES.size());
    for (int i = 0; i < EXPECTED_DISTANCES.size(); ++i) {
	node_id_t from = i / node_count;
	node_id_t to = (i + node_count) % node_count;
	QUERIES.push_back(Query{from, to});
    }

    test_route_toy_fmi(argv[1]);
    test_route_toy_ch(argv[1]);
}
