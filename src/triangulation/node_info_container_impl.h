#pragma once

#include "node_info_container.h"

#include <vector>
#include <span>

template<typename EdgeId, typename IntraEdgeId, typename Info, typename Derived>
Info node_info_container<EdgeId, IntraEdgeId, Info, Derived>::node_info(edge_id_type edge_id,
                                                                             intra_edge_id_type intra_edge_id) const {
    return static_cast<const Derived&>(*this).node_info(edge_id, intra_edge_id);
}

template<typename EdgeId, typename IntraEdgeId, typename Info, typename Derived>
std::span<const Info>
node_info_container<EdgeId, IntraEdgeId, Info, Derived>::node_infos(edge_id_type edge_id) const {
    return static_cast<const Derived&>(*this).node_infos(edge_id);
}

template<typename EdgeId, typename IntraEdgeId, typename Info, typename Derived>
void node_info_container<EdgeId, IntraEdgeId, Info, Derived>::reset(edge_id_type edge_id) {
    return static_cast<Derived&>(*this).reset(edge_id);
}

template<typename EdgeId, typename IntraEdgeId, typename Info, typename Derived>
void node_info_container<EdgeId, IntraEdgeId, Info, Derived>::reset() {
    return static_cast<Derived&>(*this).reset();
}

template<typename EdgeId, typename IntraEdgeId, typename Info, typename Derived>
void
node_info_container<EdgeId, IntraEdgeId, Info, Derived>::reset(edge_id_type edge_id, intra_edge_id_type intra_edge_id) {
    return static_cast<Derived&>(*this).reset(edge_id, intra_edge_id);
}

template<typename EdgeId, typename IntraEdgeId, typename Info, typename Derived>
std::span<Info> node_info_container<EdgeId, IntraEdgeId, Info, Derived>::node_infos(edge_id_type edge_id) {
    return static_cast<Derived&>(*this).node_infos(edge_id);
}

template<typename EdgeId, typename IntraEdgeId, typename Info, typename Derived>
Info &node_info_container<EdgeId, IntraEdgeId, Info, Derived>::node_info(edge_id_type edge_id,
                                                                              intra_edge_id_type intra_edge_id) {
    return static_cast<Derived&>(*this).node_info(edge_id, intra_edge_id);
}


template<typename EdgeId, typename IntraEdgeId, typename Info, typename Derived>
size_t node_info_container<EdgeId, IntraEdgeId, Info, Derived>::node_count(edge_id_type edge_id) const {
    return static_cast<const Derived&>(*this).node_count(edge_id);
}

template<typename EdgeId, typename IntraEdgeId, typename Info, typename Derived>
size_t node_info_container<EdgeId, IntraEdgeId, Info, Derived>::edge_count() const {
    return static_cast<const Derived&>(*this).edge_count();
}
