#include "file-io/graph_reader.h"
#include "test.h"

template<typename N, typename E>
bool
check(const graph_t<N, E> &graph) {
    bool correct = true;
    for (int e = 0; e < graph.edge_count(); ++e) {
        auto src0 = graph.forward().source(e);
        auto dest0 = graph.forward().destination(e);

        auto e_inv = graph.backward().edge_index(dest0, src0);
        auto dest1 = graph.backward().destination(e_inv);

        if (src0 != dest1) {
            correct = false;
            std::cout << " edge (" << src0 << "," << dest0 << " has no equivalent in backward adjacency list" << std::endl;
        }
    }

    // check linking of nodes in forward and backward adjacency lists
    for (int n = 0; n < graph.node_count(); ++n) {
        auto forward_id = n;
        auto backward_id = graph.backward_node_id(n);
        auto forward_id1 = graph.forward_node_id(backward_id);

        if (forward_id != forward_id1){
            correct = false;
            std::cout << " node (" << forward_id << " does not match a node in the backward graph" << std::endl;
        }

    }

    return correct;
}


void
test_read_toy_fmi(const std::string &path) {
    std::string filename = path + "/toy.fmi";

    std::vector<int> expected_offsets_fwd{ 0, 3, 3, 6, 7, 9, 9 };
    std::vector<adjacency_list_edge_t<edge_t> > expected_edges_fwd{
        adjacency_list_edge_t<edge_t>(0, 1, edge_t{ 9 }),
        adjacency_list_edge_t<edge_t>(0, 2, edge_t{ 8 }),
        adjacency_list_edge_t<edge_t>(0, 4, edge_t{ 7 }),
        adjacency_list_edge_t<edge_t>(2, 0, edge_t{ 6 }),
        adjacency_list_edge_t<edge_t>(2, 1, edge_t{ 5 }),
        adjacency_list_edge_t<edge_t>(2, 4, edge_t{ 4 }),
        adjacency_list_edge_t<edge_t>(3, 2, edge_t{ 3 }),
        adjacency_list_edge_t<edge_t>(4, 1, edge_t{ 2 }),
        adjacency_list_edge_t<edge_t>(4, 3, edge_t{ 1 })
    };

    std::vector<int> expected_offsets_bwd{ 0, 1, 4, 6, 7, 9, 9 };
    std::vector<adjacency_list_edge_t<edge_t> > expected_edges_bwd{
        adjacency_list_edge_t<edge_t>(0, 2, edge_t{ 6 }),
        adjacency_list_edge_t<edge_t>(1, 0, edge_t{ 9 }),
        adjacency_list_edge_t<edge_t>(1, 2, edge_t{ 5 }),
        adjacency_list_edge_t<edge_t>(1, 4, edge_t{ 2 }),
        adjacency_list_edge_t<edge_t>(2, 0, edge_t{ 8 }),
        adjacency_list_edge_t<edge_t>(2, 3, edge_t{ 3 }),
        adjacency_list_edge_t<edge_t>(3, 4, edge_t{ 1 }),
        adjacency_list_edge_t<edge_t>(4, 0, edge_t{ 7 }),
        adjacency_list_edge_t<edge_t>(4, 2, edge_t{ 4 })
    };

    auto c = [&](const std_graph_t &g) {
        assert_adjacency_list_equal(g.forward(), 6, 9, expected_offsets_fwd, expected_edges_fwd);
        assert_adjacency_list_equal(g.backward(), 6, 9, expected_offsets_bwd, expected_edges_bwd);
        return check(g);
    };

    test_read<fmi_format, node_t, edge_t>(filename, c);
}

void
test_read_toy_ch(const std::string &path) {
    std::string filename = path + "/toy.sch";

    std::vector<int> expected_offsets_fwd{ 0, 3, 3, 6, 8, 10, 10 };
    std::vector<adjacency_list_edge_t<ch_edge_t> > expected_edges_fwd{
        adjacency_list_edge_t<ch_edge_t>(0, 1, ch_edge_t{ 9, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(0, 2, ch_edge_t{ 8, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(0, 4, ch_edge_t{ 7, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(2, 0, ch_edge_t{ 6, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(2, 1, ch_edge_t{ 5, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(2, 4, ch_edge_t{ 4, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(3, 2, ch_edge_t{ 3, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(3, 4, ch_edge_t{ 7, 6, 5 }),
        adjacency_list_edge_t<ch_edge_t>(4, 1, ch_edge_t{ 2, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(4, 3, ch_edge_t{ 1, -1, -1 })
    };

    std::vector<int> expected_offsets_bwd{ 0, 1, 4, 6, 7, 10, 10 };
    std::vector<adjacency_list_edge_t<ch_edge_t> > expected_edges_bwd{
        adjacency_list_edge_t<ch_edge_t>(0, 2, ch_edge_t{ 6, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(1, 0, ch_edge_t{ 9, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(1, 2, ch_edge_t{ 5, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(1, 4, ch_edge_t{ 2, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(2, 0, ch_edge_t{ 8, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(2, 3, ch_edge_t{ 3, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(3, 4, ch_edge_t{ 1, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(4, 0, ch_edge_t{ 7, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(4, 2, ch_edge_t{ 4, -1, -1 }),
        adjacency_list_edge_t<ch_edge_t>(4, 3, ch_edge_t{ 7, 8, 5 })
    };

    auto c = [&](const ch_graph_t &g) {
        assert_adjacency_list_equal(g.forward(), 6, 10, expected_offsets_fwd, expected_edges_fwd);
        assert_adjacency_list_equal(g.backward(), 6, 10, expected_offsets_bwd, expected_edges_bwd);
        return check(g);
    };

    test_read<fmi_format, ch_node_t, ch_edge_t>(filename, c);
}

void
test_read_stgtregbz_fmi(const std::string &path) {
    std::string filename = path + "/stgtregbz.fmi";

    auto c = [&](const std_graph_t &g) { return check(g); };
    test_read<fmi_format, node_t, edge_t>(filename, c);
}

void
test_read_stgtregbz_ch(const std::string &path) {
    std::string filename = path + "/stgtregbz.sch";

    auto c = [&](const ch_graph_t &g) { return check(g); };
    test_read<fmi_format, ch_node_t, ch_edge_t>(filename, c);
}

int
main(int argc, char const *argv[]) {
    test_read_toy_fmi(argv[1]);
    test_read_toy_ch(argv[1]);

    test_read_stgtregbz_fmi(argv[1]);
    test_read_stgtregbz_ch(argv[1]);
}
