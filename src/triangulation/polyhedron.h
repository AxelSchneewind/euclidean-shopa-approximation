#pragma once

#include "../graph/base_types.h"
#include "../routing/dijkstra_concepts.h"

#include "../util/set_minus.h"
#include "../util/remove_duplicates.h"

/**
 * stores the topology of a polyhedron. Provides O(1) access to adjacent edges for given nodes/edges
 */
template<Topology BaseGraph, std::size_t MaxNodesPerFace>
class polyhedron {
public:
    using node_id_type = typename BaseGraph::node_id_type;
    using edge_id_type = typename BaseGraph::edge_id_type;
    using face_id_type = int;

    // edges that are part of a face
    static constexpr std::size_t EDGE_COUNT_PER_FACE = MaxNodesPerFace;
    static constexpr std::size_t FACE_COUNT_PER_EDGE = 2;

    struct edges_iterator_type {
    private:
        unsigned char face_index;
        unsigned char edge_index;
        unsigned char face_count;
        std::array<std::span<const int, EDGE_COUNT_PER_FACE>, 2> faces;

    public:
        edges_iterator_type(const polyhedron<BaseGraph, MaxNodesPerFace>&poly, std::array<face_id_type, 2> faces)
            : face_index(0),
              edge_index(0),
              face_count((!is_none(faces[0])) + (!is_none(faces[1]))),
              faces{std::span(poly._face_info[!is_none(faces[0]) ? faces[0] : 0]), std::span(poly._face_info[!is_none(faces[1]) ? faces[1] : 0])} {
        };

        edges_iterator_type& begin() { return *this; };

        struct end_type {
        };

        end_type end() const { return end_type{}; };

        bool operator==(edges_iterator_type other) const {
            return faces == other.faces;
        }

        bool operator==(end_type) const {
            return face_index == face_count;
        }

        bool operator!=(edges_iterator_type other) const {
            return faces != other.faces;
        }

        bool operator!=(end_type) const {
            return face_index != face_count;
        }

        edges_iterator_type& operator++() {
            edges_iterator_type&result = *this;

            edge_index++;
            if (edge_index >= EDGE_COUNT_PER_FACE) {
                edge_index = 0;
                face_index++;
            }

            return result;
        };

        edges_iterator_type& operator++(int) {
            edge_index++;
            if (edge_index >= EDGE_COUNT_PER_FACE) {
                edge_index = 0;
                face_index++;
            }
            return *this;
        };

        edge_id_type operator*() {
            return faces[face_index][edge_index];
        }
    };

private:
    static_assert(sizeof(std::array<edge_id_type, EDGE_COUNT_PER_FACE>) == EDGE_COUNT_PER_FACE * sizeof(edge_id_type));

    using edge_info_type = std::array<face_id_type, FACE_COUNT_PER_EDGE>;
    using edge_link_type = std::array<edge_id_type, (EDGE_COUNT_PER_FACE - 1) * FACE_COUNT_PER_EDGE>;

    std::size_t _boundary_node_count;
    std::size_t _boundary_edge_count;

    [[gnu::hot]]
    std::vector<bool> _is_boundary_edge;
    [[gnu::hot]]
    std::vector<bool> _is_boundary_node;

    // for each triangle
    std::vector<std::array<edge_id_type, EDGE_COUNT_PER_FACE>> _face_info;

    // for each edge
    std::vector<edge_info_type> _edge_info;
    [[gnu::hot]]
    std::vector<edge_link_type> _edge_links;

    // for each node, store edges that are reachable by crossing a face
    [[gnu::hot]]
    std::vector<edge_id_type> _node_edges;
    [[gnu::hot]]
    std::vector<int> _node_edges_offsets;


    polyhedron(std::vector<std::array<edge_id_type, EDGE_COUNT_PER_FACE>>&&adjacent_edges,
               std::vector<std::array<face_id_type, FACE_COUNT_PER_EDGE>>&&adjacent_faces,
               std::vector<bool>&& is_boundary_node,
               std::vector<bool>&& is_boundary_edge,
               std::vector<edge_id_type>&&node_edges,
               std::vector<int>&&node_edge_offsets);

public:
    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(edge_info_type);

    std::size_t node_count() const { return _node_edges_offsets.size() - 1; }

    std::size_t edge_count() const { return _edge_info.size(); }

    std::size_t face_count() const { return _face_info.size(); }

    std::size_t boundary_edge_count() const {
        return _boundary_node_count;
   }

    bool is_boundary_edge(node_id_type node) const {
        return _is_boundary_edge[node];
    }

    bool is_boundary_node(node_id_type node) const {
        return _is_boundary_node[node];
    }

    /**
     * @return
     */
    static polyhedron make_polyhedron(BaseGraph const&triangulation_edges,
                                    std::vector<std::array<node_id_type, MaxNodesPerFace>> &&faces);

    std::span<const face_id_type, FACE_COUNT_PER_EDGE> edge_faces(edge_id_type edge) const {
        return {_edge_info[edge]};
    }

    std::span<const edge_id_type, EDGE_COUNT_PER_FACE> face_edges(face_id_type face) const {
        return {_face_info[face]};
    }

    std::span<const edge_id_type, std::dynamic_extent> node_edges(node_id_type node) const {
        return {
            _node_edges.begin() + _node_edges_offsets[node],
            _node_edges.begin() + _node_edges_offsets[node + 1]
        };
    }

    /**
     * gets edges that belong to the faces bordering this edge
     * @param edge
     * @return
     */
    std::span<const edge_id_type, std::dynamic_extent> edges(edge_id_type edge) const {
        assert(!is_none(edge));
        if (_is_boundary_edge[edge]) {
            assert(is_none(_edge_links[edge][EDGE_COUNT_PER_FACE - 1]));
            return {_edge_links[edge].begin(), _edge_links[edge].begin() + (EDGE_COUNT_PER_FACE - 1)};
        } else {
            return {_edge_links[edge].begin(), _edge_links[edge].end()};
        }
    };
};
