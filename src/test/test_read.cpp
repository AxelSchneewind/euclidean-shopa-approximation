#include "../file-io/fmi_file_io_impl.h"
#include "test.h"


void
test_read_toy_fmi(const std::string &path) {
    std::string filename = path + "/toy.fmi";

    std::vector<int> expected_offsets_fwd{ 0, 3, 3, 6, 7, 9, 9 };
    std::vector<adjacency_list_edge<node_id_t, edge_t>> expected_edges_fwd{
            {0, 1, edge_t{ 9 }}, {0, 2, edge_t{ 8 }},
            {0, 4, edge_t{ 7 }}, {2, 0, edge_t{ 6 }},
      {2, 1, edge_t{ 5 }}, {2, 4, edge_t{ 4 }},
      {3, 2, edge_t{ 3 }}, {4, 1, edge_t{ 2 }},
      {4, 3, edge_t{ 1 }}
    };

    std::vector<int> expected_offsets_bwd{ 0, 1, 4, 6, 7, 9, 9 };
    std::vector<adjacency_list_edge<node_id_t, edge_t> > expected_edges_bwd{
      {0, 2, edge_t{ 6 }}, {1, 0, edge_t{ 9 }},
      {1, 2, edge_t{ 5 }}, {1, 4, edge_t{ 2 }},
      {2, 0, edge_t{ 8 }}, {2, 3, edge_t{ 3 }},
      {3, 4, edge_t{ 1 }}, {4, 0, edge_t{ 7 }},
      {4, 2, edge_t{ 4 }}
    };

    auto c = [&](const std_graph_t &g) {
        const size_t node_count = 6;
	const size_t edge_count = 9;
        assert_adjacency_list_equal(g.topology(), node_count, edge_count, expected_offsets_fwd, expected_edges_fwd);
        assert_adjacency_list_equal(g.inverse_topology(), node_count, edge_count, expected_offsets_bwd, expected_edges_bwd);
        return true;
    };

    test_read<fmi_file_io, std_graph_t, decltype(c)>(filename, c);
}

void
test_read_toy_ch(const std::string &path) {
    std::string filename = path + "/toy.sch";

    std::vector<int> expected_offsets_fwd{ 0, 3, 3, 6, 8, 10, 10 };
    std::vector<adjacency_list_edge<node_id_t, ch_edge_t> > expected_edges_fwd{
            {0, 1, ch_edge_t{ 9, -1, -1 }},
      {0, 2, ch_edge_t{ 8, -1, -1 }},
      {0, 4, ch_edge_t{ 7, -1, -1 }},
      {2, 0, ch_edge_t{ 6, -1, -1 }},
      {2, 1, ch_edge_t{ 5, -1, -1 }},
      {2, 4, ch_edge_t{ 4, -1, -1 }},
      {3, 2, ch_edge_t{ 3, -1, -1 }},
      {3, 4, ch_edge_t{ 7, 6, 5 }},
      {4, 1, ch_edge_t{ 2, -1, -1 }},
      {4, 3, ch_edge_t{ 1, -1, -1 }}
    };

    std::vector<int> expected_offsets_bwd{ 0, 1, 4, 6, 7, 10, 10 };
    std::vector<adjacency_list_edge<node_id_t, ch_edge_t> > expected_edges_bwd{
      {0, 2, ch_edge_t{ 6, -1, -1 }},
      {1, 0, ch_edge_t{ 9, -1, -1 }},
      {1, 2, ch_edge_t{ 5, -1, -1 }},
      {1, 4, ch_edge_t{ 2, -1, -1 }},
      {2, 0, ch_edge_t{ 8, -1, -1 }},
      {2, 3, ch_edge_t{ 3, -1, -1 }},
      {3, 4, ch_edge_t{ 1, -1, -1 }},
      {4, 0, ch_edge_t{ 7, -1, -1 }},
      {4, 2, ch_edge_t{ 4, -1, -1 }},
      {4, 3, ch_edge_t{ 7, 8, 5 }}
    };

    auto c = [&](const ch_graph_t &g) {
      	const size_t node_count = 6;
      	const size_t edge_count = 10;
        assert_adjacency_list_equal(g.topology(), node_count, edge_count, expected_offsets_fwd, expected_edges_fwd);
        assert_adjacency_list_equal(g.inverse_topology(), node_count, edge_count, expected_offsets_bwd, expected_edges_bwd);
        return true;
    };

    test_read<fmi_file_io, ch_graph_t, decltype(c)>(filename, c);
}

void
test_read_stgtregbz_fmi(const std::string &path) {
    std::string filename = path + "/stgtregbz/stgtregbz.fmi";

    auto c = [&](const std_graph_t &g) { return true; };
    test_read<fmi_file_io, std_graph_t, decltype(c)>(filename, c);
}

void
test_read_stgtregbz_ch(const std::string &path) {
    std::string filename = path + "/stgtregbz/stgtregbz.sch";

    auto c = [&](const ch_graph_t &g) { return true; };
    test_read<fmi_file_io, ch_graph_t, decltype(c)>(filename, c);
}

int
main(int argc, char const *argv[]) {
    test_read_toy_fmi(argv[1]);
    test_read_toy_ch(argv[1]);

    test_read_stgtregbz_fmi(argv[1]);
    test_read_stgtregbz_ch(argv[1]);
}
