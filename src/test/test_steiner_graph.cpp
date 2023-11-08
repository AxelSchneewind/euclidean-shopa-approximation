
#include <string>
#include <istream>
#include <fstream>


void
test_toy(const std::string &path) {
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
