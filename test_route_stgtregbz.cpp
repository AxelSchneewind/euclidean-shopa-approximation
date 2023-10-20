#include "file-io/graph_reader.h"
#include "test.h"

static std::vector<distance_t> expected_distances;
static std::vector<Query> queries;

void
test_route_stgtregbz_fmi(std::string path) {
	std::string filename = path + "/stgtregbz.fmi";
	std::ifstream input;
	input.open(filename);
	fmi_format reader;
	std_graph_t graph(reader.read<node_t, edge_t>(input));
    std_routing_t router(&graph);

    test_routing(graph, router, queries, expected_distances);

	std::cout << "routing in " << filename << " successful" << std::endl;
    return;
};

void
test_route_stgtregbz_ch(std::string path) {
	std::string filename = path + "/stgtregbz.sch";
	std::ifstream input;
	input.open(filename);

    std::cout << "reading from " << filename << std::flush;

	fmi_format reader;
	ch_graph_t graph(reader.read<ch_node_t, ch_edge_t>(input));
    ch_routing_t router(&graph);

    std::cout << " done" << std::endl;

    test_routing(graph, router, queries, expected_distances);

	std::cout << "routing in " << filename << " successful" << std::endl;
    return;
};


int main(int argc, char const *argv[])
{
    std::ifstream query_stream;
    query_stream.open(std::string(argv[2]) + "/stgtregbz.que");

    std::ifstream distances;
    distances.open(std::string(argv[2]) + "/stgtregbz.sol");

    queries = get_queries(query_stream);
    expected_distances = get_distances(distances);
    test_route_stgtregbz_ch(argv[1]);
    test_route_stgtregbz_fmi(argv[1]);
}

