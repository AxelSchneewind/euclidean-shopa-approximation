#pragma once

#include "../graph/adjacency_list.h"

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
        char face_index;
        char edge_index;
        char face_count;
        std::array<std::span<const int, EDGE_COUNT_PER_FACE>, 2> faces;

    public:
        edges_iterator_type(const polyhedron<BaseGraph, MaxNodesPerFace> &__poly, std::array<face_id_type, 2> __faces)
                : faces(
                {std::span(__poly._M_face_info[__faces[0]]), std::span(__poly._M_face_info[__faces[1]])}),
                  edge_index(0),
                  face_index(0),
                  face_count(1 + (!is_none(__faces[1]) ? 1 : 0)) {
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

        bool operator!=(end_type __other) const {
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

    polyhedron(
            std::vector<std::array<edge_id_t, EDGE_COUNT_PER_FACE>> &&__adjacent_edges,
            std::vector<std::array<edge_id_t, FACE_COUNT_PER_EDGE>> &&__adjacent_faces,
            std::vector<edge_id_t> &&__inverse_edges)
            : _M_face_info(std::move(__adjacent_edges)),
              _M_edge_info() {
        for (int i = 0; i < __adjacent_faces.size(); ++i) {
            edge_info_type info = {__adjacent_faces.back(), __inverse_edges.back()};
            _M_edge_info.push_back(info);

            __adjacent_edges.pop_back();
            __inverse_edges.pop_back();

            if (i % (1024 * 1024) == 0) {
                __adjacent_edges.shrink_to_fit();
                __inverse_edges.shrink_to_fit();
            }
        }

        list_invert(_M_edge_info);
    };
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

    // /**
    //  * gets edges that belong to the edge_faces bordering this edge
    //  * does not include the inverse to the given edge
    //  * @param __edge
    //  * @return
    //  */
    // std::vector<edge_id_type> edges(int __edge) const { // TODO not return a vector but e.g. pair of spans
    //     // std::span<const edge_id_type, EDGE_COUNT_PER_FACE> edges(int __edge) const {
    //     std::vector<edge_id_type> result(2 * EDGE_COUNT_PER_FACE);

    //     int j = 0;

    //     for (int i = 0; i < EDGE_COUNT_PER_FACE; ++i) {
    //         result[j++] = _M_face_info[_M_adjacent_faces[__edge][0]][i];
    //     }

    //     if (!is_none(_M_adjacent_faces[__edge][1]))
    //         for (int i = 0; i < EDGE_COUNT_PER_FACE; ++i) {
    //             result[j++] = _M_face_info[_M_adjacent_faces[__edge][1]][i];
    //         }

    //     result.resize(j);
    //     return result;
    // };

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


    /**
     * gets edges that belong to the edge_faces bordering this node
     * @param __node
     * @return
     */
    // TODO get access to offset array
    std::span<const edge_id_type, std::dynamic_extent> node_edges(node_id_type __node) const {
        throw;
        //return std::span(_M_face_info.edge(__node), _M_face_info.edge(__node + 1));
    };
};

