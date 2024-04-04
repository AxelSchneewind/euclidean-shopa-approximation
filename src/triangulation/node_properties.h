#pragma once

#include <concepts>
#include <ranges>
#include <unordered_map>
#include <cassert>

template <typename T>
concept sized_random_access_range = std::ranges::random_access_range<T> && std::ranges::sized_range<T>;

template <sized_random_access_range NodeRange, sized_random_access_range FaceRange, sized_random_access_range PropertyRange>
requires requires (typename PropertyRange::value_type t) {
    { t.out_degree } -> std::convertible_to<bool>;
    { t.in_degree } -> std::convertible_to<bool>;
}
void compute_degrees(NodeRange const& nodes, FaceRange const& faces, PropertyRange& properties) {
    size_t node_count = std::ranges::size(nodes);
    size_t face_count = std::ranges::size(faces);
    assert(std::ranges::size(properties) == node_count);

    // mark nodes as connected and increment out and in degree
    for (size_t t = 0; t < face_count; ++t) {
        auto&& triangle = faces[t];
        properties[triangle[0]].out_degree++;
        properties[triangle[1]].out_degree++;
        properties[triangle[2]].out_degree++;
        properties[triangle[0]].in_degree++;
        properties[triangle[1]].in_degree++;
        properties[triangle[2]].in_degree++;
    }
}

namespace std {
    template<>
    struct hash<pair<long, long>> {
        size_t operator()(pair<long, long> const& p) const {
            return (hash<long>{}(p.first) << 1) ^ hash<long>{}(p.second);
        }
    };
}

template <sized_random_access_range NodeRange, sized_random_access_range FaceRange, typename PropertyRange>
requires requires (typename PropertyRange::mapped_type t) {
    { t.is_boundary_edge } -> std::convertible_to<bool>;
}
void compute_boundary_edge(NodeRange const& nodes, FaceRange const& faces, PropertyRange& properties) {
    size_t node_count = std::ranges::size(nodes);
    size_t face_count = std::ranges::size(faces);
    assert(std::ranges::size(properties) == node_count);

    // count faces for each edge
    std::unordered_map<std::pair<long, long>, char> adjacent_face_count;
    for (size_t t = 0; t < face_count; ++t) {
        auto&& triangle = faces[t];

        adjacent_face_count[{triangle[0], triangle[1]}]++;
        adjacent_face_count[{triangle[1], triangle[2]}]++;
        adjacent_face_count[{triangle[2], triangle[0]}]++;
        adjacent_face_count[{triangle[0], triangle[2]}]++;
        adjacent_face_count[{triangle[1], triangle[0]}]++;
        adjacent_face_count[{triangle[2], triangle[1]}]++;
    }

    // if only one face adjacent to an edge, mark nodes as boundary nodes
    for (auto [node_pair, adjacent_faces] : adjacent_face_count) {
        assert(adjacent_faces == 1 || adjacent_faces == 2);
        if (adjacent_faces == 1) {
            properties[{node_pair.first, node_pair.second}].is_boundary_edge = true;
        }
    }
}


template <sized_random_access_range NodeRange, sized_random_access_range FaceRange, sized_random_access_range PropertyRange>
requires requires (typename PropertyRange::value_type t) {
    { t.is_boundary_node } -> std::convertible_to<bool>;
}
void compute_boundary_node(NodeRange const& nodes, FaceRange const& faces, PropertyRange& properties) {
    size_t node_count = std::ranges::size(nodes);
    size_t face_count = std::ranges::size(faces);
    assert(std::ranges::size(properties) == node_count);

    // count faces for each edge
    std::unordered_map<std::pair<long, long>, char> adjacent_face_count;
    for (size_t t = 0; t < face_count; ++t) {
        auto&& triangle = faces[t];

        if constexpr (requires (typename PropertyRange::value_type v) { {v.connected} -> std::convertible_to<bool>; }) {
            properties[triangle[0]].connected = true;
            properties[triangle[1]].connected = true;
            properties[triangle[2]].connected = true;
        }

        adjacent_face_count[{triangle[0], triangle[1]}]++;
        adjacent_face_count[{triangle[1], triangle[2]}]++;
        adjacent_face_count[{triangle[2], triangle[0]}]++;
        adjacent_face_count[{triangle[0], triangle[2]}]++;
        adjacent_face_count[{triangle[1], triangle[0]}]++;
        adjacent_face_count[{triangle[2], triangle[1]}]++;
    }

    // if only one face adjacent to an edge, mark nodes as boundary nodes
    for (auto [node_pair, adjacent_faces] : adjacent_face_count) {
        assert(adjacent_faces == 1 || adjacent_faces == 2);
        if (adjacent_faces == 1) {
            long node1 = node_pair.first;
            long node2 = node_pair.second;
            properties[node1].is_boundary_node = true;
            properties[node2].is_boundary_node = true;
        }
    }
}
