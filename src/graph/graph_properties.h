

namespace graph_properties {

    /**
     * checks that (v, w) in graph => (w,v) in graph
     * @tparam G
     * @param graph
     * @return
     */
    template <typename G>
    static bool is_bidirectional(const G& graph) {
        bool result = true;

        for (auto edge : graph.edge_ids()) {
            typename G::node_id_type node1 = graph.source(edge);
            typename G::node_id_type node2 = graph.destination(edge);
            result &= graph.has_edge(node2, node1);

            if (!result)
                break;
        }

        return result;
    }

}