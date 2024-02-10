#pragma once

#include "node_info_container.h"
#include <vector>

template<typename EdgeId, typename IntraEdgeId, typename Info>
class node_info_array
        : public node_info_container<EdgeId, IntraEdgeId, Info, node_info_array<EdgeId, IntraEdgeId, Info>> {
public:
    using edge_id_type = EdgeId;
    using intra_edge_id_type = IntraEdgeId;
    using info_type = Info;


private:
    using index_type = size_t;

    std::vector<Info> node_info_list;
    std::vector<index_type> offsets;
    Info default_value;

    index_type offset(edge_id_type edge_id) const;

    index_type offset_next(edge_id_type edge_id) const;

public:
    ~node_info_array() = default;

    node_info_array(std::vector<index_type> &&offsets, Info default_value);

    node_info_array(std::vector<index_type> const &offsets, Info default_value);

    node_info_array(node_info_array &&) noexcept = default;

    node_info_array &operator=(node_info_array &&) = default;

    info_type &node_info(edge_id_type edge_id, intra_edge_id_type intra_edge_id);

    info_type node_info(edge_id_type edge_id, intra_edge_id_type intra_edge_id) const;

    std::span<info_type> node_infos(edge_id_type edge_id);

    std::span<const info_type> node_infos(edge_id_type edge_id) const;

    void reset(edge_id_type edge_id, intra_edge_id_type intra_edge_id);

    void reset(edge_id_type edge_id);

    void reset();

    size_t node_count(edge_id_type edge_id) const;

    size_t edge_count() const;
};
