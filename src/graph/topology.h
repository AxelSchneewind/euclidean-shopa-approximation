#include <span>
#include <vector>
#include "../util/counting_iterator.h"


#include <coroutine>

template<typename NodeId, typename EdgeId> requires std::convertible_to<NodeId, size_t> && std::convertible_to<EdgeId, size_t>
struct undirected_offset_list {
public:
    using node_id_type = NodeId;
    using edge_id_type = EdgeId;

private:
    std::vector<edge_id_type> out;

public:
    counter<edge_id_type> out_edges(const node_id_type __node) {
        return {out[__node], out[__node + 1]};
    }

    counter<edge_id_type> in_edges(const node_id_type __node) {
        return {out[__node], out[__node + 1]};
    }
};

template<typename NodeId, typename EdgeId> requires std::convertible_to<NodeId, size_t> && std::convertible_to<EdgeId, size_t>
struct offset_list {
public:
    using node_id_type = NodeId;
    using edge_id_type = EdgeId;

private:
    std::vector<edge_id_type> out;
    std::vector<edge_id_type> in;

public:
    counter<edge_id_type> out_edges(const node_id_type __node) {
        return {out[__node], out[__node + 1]};
    }

    counter<edge_id_type> in_edges(const node_id_type __node) {
        return {in[__node], in[__node + 1]};
    }
};


template<typename NodeId, typename EdgeId, typename EdgeLinks, typename Steiner>
struct steiner_topology {
public:
    using node_id_type = NodeId;
    using edge_id_type = EdgeId;

private:
    std::shared_ptr<EdgeLinks> edge_links;
    std::shared_ptr<Steiner> steiner_info;

public:
    edge_id_type out_edges(const node_id_type __node_id) {
        std::vector<edge_id_type > result;

        if (__node_id.steiner_index > 0) {
            co_yield {__node_id.edge, __node_id.steiner_index - 1};
        }
        if (__node_id.steiner_index + 1 < steiner_info.edge(__node_id.edge).node_count) {
            co_yield {__node_id.edge, __node_id.steiner_index + 1};
        } else if (__node_id.steiner_index + 1 == steiner_info.edge(__node_id.edge).node_count) {
            auto inv_id = edge_links.inverse_edge(__node_id.edge);
            co_yield {inv_id, steiner_info(inv_id).node_count - 1};
        }

        // iterate over edges of adjacent triangles
        auto dest_edges = edge_links.edges(__node_id.edge);
        for (auto dest_edge: dest_edges) {
            if (is_none(dest_edge)) {
                break;
            }

            // iterate over steiner points on destination edge
            auto info = steiner_info.edge(dest_edge);
            for (int steiner_index = 0; steiner_index < info.node_count; ++steiner_index) {
                co_yield {dest_edge, steiner_index};
            }
        }
    }

    edge_id_type in_edges(const node_id_type __node) {
        for (edge_id_type e : out_edges(__node))
            co_yield e;
    }
};
