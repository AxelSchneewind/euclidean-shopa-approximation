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

    info_type& node_info(edge_id_type edge_id, intra_edge_id_type intra_edge_id) {
        return static_cast<Derived&>(*this).node_info(edge_id, intra_edge_id);
    }

    info_type node_info(edge_id_type edge_id, intra_edge_id_type intra_edge_id) const {
        return static_cast<const Derived&>(*this).node_info(edge_id, intra_edge_id);
    }

    std::span<info_type> node_infos(edge_id_type edge_id) {
        return static_cast<Derived&>(*this).node_infos(edge_id);
    }

    std::span<const info_type> node_infos(edge_id_type edge_id) const {
        return static_cast<const Derived&>(*this).node_infos(edge_id);
    }

    void reset(edge_id_type edge_id, intra_edge_id_type intra_edge_id) {
        return static_cast<Derived&>(*this).reset(edge_id, intra_edge_id);
    }

    void reset(edge_id_type edge_id) {
        return static_cast<Derived&>(*this).reset(edge_id);
    }
};





template <typename EdgeId, typename IntraEdgeId, typename Info>
class node_info_array
        : public node_info_container<EdgeId, IntraEdgeId, Info, node_info_array<EdgeId, IntraEdgeId, Info>>{
public:
    using edge_id_type = EdgeId;
    using intra_edge_id_type = IntraEdgeId;
    using info_type = Info;


private:
    using index_type = unsigned int;

    std::vector<Info> node_info_list;
    std::vector<index_type> offsets;
    Info default_value;

    index_type offset(edge_id_type edge_id) const {
        assert(edge_id < offsets.size() - 1);
        return offsets[edge_id];
    }
    index_type offset_next(edge_id_type edge_id) const {
        assert(edge_id + 1 < offsets.size());
        return offsets[edge_id + 1];
    }

public:
    node_info_array(std::vector<index_type> && offsets, Info default_value) : offsets(std::move(offsets)), default_value(default_value), node_info_list(offsets.back(), default_value) {};
    node_info_array(std::vector<index_type> const& offsets, Info default_value) : offsets(offsets), default_value(default_value), node_info_list(offsets.back(), default_value) {};

    info_type& node_info(edge_id_type edge_id, intra_edge_id_type intra_edge_id) {
        auto of = offset(edge_id);
        return node_info_list[of + intra_edge_id];
    }

    info_type node_info(edge_id_type edge_id, intra_edge_id_type intra_edge_id) const {
        auto of = offset(edge_id);
        return node_info_list[of + intra_edge_id];
    }

    std::span<info_type> node_infos(edge_id_type edge_id) {
        auto of = offset(edge_id);
        auto ofn = offset_next(edge_id);
        return {node_info_list.begin() + of, node_info_list.begin() + ofn};
    }

    std::span<const info_type> node_infos(edge_id_type edge_id) const {
        auto of = offset(edge_id);
        auto ofn = offset_next(edge_id);
        return {node_info_list.begin() + of, node_info_list.begin() + ofn};
    }

    void reset(edge_id_type edge_id, intra_edge_id_type intra_edge_id) {
        auto of = offset(edge_id);
        node_info_list[of + intra_edge_id] = default_value;
    }

    void reset(edge_id_type edge_id) {
        auto of = offset(edge_id);
        auto ofn = offset_next(edge_id);
        std::fill(node_info_list.begin() + of, node_info_list.begin() + ofn, default_value);
    }
};