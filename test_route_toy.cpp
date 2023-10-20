#include "file-io/graph_reader.h"
#include "test.h"

static const std::vector<distance_t> expected_distances {
	0, 9, 8, 8, 7, DISTANCE_INF,
	DISTANCE_INF, 0, DISTANCE_INF, DISTANCE_INF, DISTANCE_INF, DISTANCE_INF,
	6, 5, 0, 5, 4, DISTANCE_INF,
	9, 8, 3, 0, 7, DISTANCE_INF,
	10, 2, 4, 1, 0, DISTANCE_INF,
	DISTANCE_INF, DISTANCE_INF, DISTANCE_INF, DISTANCE_INF, DISTANCE_INF, 0
};

void
test_route_toy_fmi(std::string path) {
	std::string filename = path + "/toy.fmi";
	std::ifstream input;
	input.open(filename);
	fmi_format reader;
	std_graph_t graph(reader.read<node_t, edge_t>(input));

	std_routing_t router(&graph);

	for(int i = 0; i < graph.node_count() * graph.node_count(); i++) {
		node_id_t from = i / graph.node_count();
		node_id_t to = i % graph.node_count();
		router.compute_route(from, to);
		assert_equal(expected_distances[i], router.distance());
	}

	std::cout << "routing in " << filename << " successful" << std::endl;
    return;
};

void
test_route_toy_ch(std::string path) {
	std::string filename = path + "/toy.sch";
	std::ifstream input;
	input.open(filename);
	fmi_format reader;
	ch_graph_t graph(reader.read<ch_node_t, ch_edge_t>(input));

	ch_routing_t router(&graph);

	for(int i = 0; i < graph.node_count() * graph.node_count(); i++) {
		node_id_t from = i / graph.node_count();
		node_id_t to = (i + graph.node_count()) % graph.node_count();
		router.compute_route(from, to);
		assert_equal(expected_distances[i], router.distance());
	}

	std::cout << "routing in " << filename << " successful" << std::endl;
    return;
};

int main(int argc, char const *argv[])
{
    test_route_toy_fmi(argv[1]);
    test_route_toy_ch(argv[1]);
}

