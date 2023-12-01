#pragma once

#include "../graph/base_types.h"
#include "../routing/dijkstra_concepts.h"

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
    static constexpr std::size_t EDGE_COUNT_PER_FACE = MaxNodesPerFace * 2;
    static constexpr std::size_t FACE_COUNT_PER_EDGE = 2;

    struct [[deprecated]] edges_iterator_type {
    private:
        unsigned char face_index;
        unsigned char edge_index;
        unsigned char face_count;
        std::array<std::span<const int, EDGE_COUNT_PER_FACE>, 2> faces;

    public:
        edges_iterator_type(const polyhedron<BaseGraph, MaxNodesPerFace> &__poly, std::array<face_id_type, 2> __faces)
                : face_index(0),
                  edge_index(0),
                  face_count(1 + (!is_none(__faces[1]) ? 1 : 0)),
                  faces({std::span(__poly._M_face_info[__faces[0]]), std::span(__poly._M_face_info[__faces[1]])}) {
        };

        edges_iterator_type &begin() { return *this; };

        struct end_type {
        };

        end_type end() const { return end_type{}; };

        bool operator==(edges_iterator_type __other) const {
            return faces == __other.faces;
        }

        bool operator==(end_type __other) const {
            return face_index == face_count;
        }

        bool operator!=(edges_iterator_type __other) const {
            return faces != __other.faces;
        }

        bool operator!=(end_type) const {
            return face_index != face_count;
        }

        edges_iterator_type &operator++() {
            edges_iterator_type &result = *this;

            edge_index++;
            if (edge_index >= EDGE_COUNT_PER_FACE) {
                edge_index = 0;
                face_index++;
            }

            return result;
        };

        edges_iterator_type &operator++(int) {
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

    struct edge_info_type {
        std::array<face_id_type, FACE_COUNT_PER_EDGE> adjacent_faces;
        edge_id_type inverse_edge;
    };

    // for each triangle
    std::vector<std::array<edge_id_type, EDGE_COUNT_PER_FACE>> _M_face_info;
    // for each edge
    std::vector<edge_info_type> _M_edge_info;

    polyhedron(std::vector<std::array<edge_id_type, EDGE_COUNT_PER_FACE>> &&__adjacent_edges,
               std::vector<std::array<face_id_type, FACE_COUNT_PER_EDGE>> &&__adjacent_faces,
               std::vector<edge_id_type> &&__inverse_edges);

public:

    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(edge_info_type);

    /**
     * makes a polyhedron object from the given base graph
     * @param __base
     * @return
     */
    static polyhedron<BaseGraph, MaxNodesPerFace> make_polyhedron(const BaseGraph &__base,
                                                                  const std::vector<std::array<typename BaseGraph::node_id_type, MaxNodesPerFace>> &__faces);

    std::span<const int, FACE_COUNT_PER_EDGE> edge_faces(int __edge) const {
        return {_M_edge_info[__edge].adjacent_faces};
    }

    std::span<const int, EDGE_COUNT_PER_FACE> face_edges(int __face) const {
        return {_M_face_info[__face]};
    }

    /**
     * gets edges that belong to the edge_faces bordering this edge
     * does not include the inverse to the given edge
     * @param __edge
     * @return
     */
    edges_iterator_type edges(int __edge) const {
        return {*this, {_M_edge_info[__edge].adjacent_faces}};
    };

    /**
     * gets the id of the inverse edge for the given one
     * @param __edge
     * @return
     */
    edge_id_type inverse_edge(edge_id_type __edge) const {
        return _M_edge_info[__edge].inverse_edge;
    };
};

