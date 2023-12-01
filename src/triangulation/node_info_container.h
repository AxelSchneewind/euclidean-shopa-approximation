#pragma once

#include <vector>

template <typename EdgeId, typename IntraEdgeId, typename Info, typename Derived>
class node_info_container {
public:
    using edge_id_type = EdgeId;
    using intra_edge_id_type = IntraEdgeId;
    using info_type = Info;


private:
    using index_type = unsigned int;

public:

    info_type& node_info(edge_id_type edge_id, intra_edge_id_type intra_edge_id);

    info_type node_info(edge_id_type edge_id, intra_edge_id_type intra_edge_id) const;

    std::span<info_type> node_infos(edge_id_type edge_id);

    std::span<const info_type> node_infos(edge_id_type edge_id) const;

    void reset(edge_id_type edge_id, intra_edge_id_type intra_edge_id);

    void reset(edge_id_type edge_id);
};