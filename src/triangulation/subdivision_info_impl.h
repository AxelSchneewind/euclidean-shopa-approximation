#include "subdivision_table.h"

#include "../graph/base_types.h"

#include <vector>


std::vector<float>
subdivision_table::min_r_per_triangle_class(const std::vector<node_t> &__nodes, const std::vector<float> &r_values,
                                            const std::vector<triangle> &__faces) {
    std::vector<float> min_r(M_PI / step_size, infinity<float>);

    for (auto face: __faces) {
        for (int node_index = 0; node_index < 3; ++node_index) {
            auto c1 = __nodes[face[node_index]].coordinates;
            auto c2 = __nodes[face[(node_index + 1) % 3]].coordinates;
            auto c3 = __nodes[face[(node_index + 2) % 3]].coordinates;
            float radians = angle(c1, c2, c1, c3);
            int index = class_index(radians);

            float r_relative = r_values[face[node_index]] / distance(c2, c1);
            min_r[index] = std::min(min_r[index], r_relative);
        }
    }

    return min_r;
}

std::vector<subdivision_table::edge_class>
subdivision_table::precompute(float __epsilon, float __min_relative_r_value) {
    std::vector<edge_class> triangle_classes;

    for (int index = 0; index < step_count; ++index) {
        float angle = class_angle(index);

        triangle_classes.emplace_back();

        float sine = std::sin(angle);

        float relative = __min_relative_r_value;
        float edge_distance = 0;

        while (relative < 1) {
            triangle_classes.back().node_positions.push_back(relative);
            edge_distance = sine * relative;
            relative += __epsilon * edge_distance;
        }
    }

    return triangle_classes;
}

std::vector<subdivision_table::subdivision_edge_info>
subdivision_table::make_subdivision_info(const adjacency_list<int, std::nullptr_t> &__triangulation,
                                         const std::vector<node_t> &__nodes,
                                         const polyhedron<adjacency_list<int, std::nullptr_t>, 3> &__polyhedron,
                                         const std::vector<edge_class> &__table,
                                         const std::vector<std::float16_t> &__r_values, float __epsilon) {

    // store subdivision information here
    std::vector<subdivision_edge_info> result;
    result.reserve(__triangulation.edge_count());

    for (size_t i = 0; i < __triangulation.edge_count(); i++) {
        auto node1 = __triangulation.source(i);
        auto node2 = __triangulation.destination(i);

        auto inv_edge = __triangulation.edge_id(node2, node1);

        // get coordinates
        auto c1 = __nodes[node1].coordinates;
        auto c2 = __nodes[node2].coordinates;

        // get minimal angle for node1 and node2
        // treat angles > 90 degrees like 90 degrees
        float angle1 = M_PI / 2; // between node1->node2 and node1->node3
        float angle2 = M_PI / 2; // between node2->node1 and node2->node3

        for (auto edge: __polyhedron.edges(i)) {
            if (is_none(edge)) continue;

            auto node3 = __triangulation.destination(edge);
            if (node3 == node1 || node3 == node2) continue;

            auto c3 = __nodes[node3].coordinates;

            // compute angles
            auto a1 = angle(c1, c2, c1, c3);
            auto a2 = angle(c2, c1, c2, c3);
            if (a1 < angle1)
                angle1 = a1;
            if (a2 < angle2)
                angle2 = a2;
        }
        angle1 = std::max(angle1, min_angle);
        angle2 = std::max(angle2, min_angle);
        angle1 = std::min(angle1, max_angle);
        angle2 = std::min(angle2, max_angle);

        // length of the edge
        double length = distance(c2, c1);

        // relative value where the mid-point (with max distance to other edges) lies between node1 and node2
        float mid_value = 1 / (1 + std::sin(angle1) / std::sin(angle2));
        assert(mid_value < 1 && mid_value > 0);

        // store minimal distance from the given mid-point to any other edge, relative to this edges length
        float mid_dist = mid_value * std::sin(angle1);

        // distance values have been computed already, convert to r(v) relative to this edges length
        std::float16_t r = (__epsilon / 5) * __r_values[node1] / (mid_value * length);
        assert(r >= 0);
        // assert(r < __epsilon / 5);

        // get the class this edge belongs to
        auto index = class_index(angle1);

        // get interval in node_positions that is between r and mid_value
        size_t first = 0;
        auto &node_positions = __table[index].node_positions;
        while (first < node_positions.size() && node_positions[first] < r)
            first++;

        size_t last = first;
        while (last < node_positions.size() && node_positions[last] < mid_value)
            last++;

        assert((size_t) std::numeric_limits<unsigned char>::max > first);
        assert((size_t) std::numeric_limits<unsigned char>::max > last);
        assert(first < last + 3);
        assert(last - first >= 0);

        auto entry = subdivision_edge_info{static_cast<std::float16_t>(mid_value),
                // static_cast<std::float16_t>(mid_dist),
                                           r,
                                           static_cast<unsigned char>(last - first + 3UL),
                                           static_cast<unsigned char>(index),
                                           static_cast<unsigned char>(first)};
        result.push_back(entry);
    }

    return result;
}

subdivision_table::subdivision_table(subdivision_table &&__other) noexcept: triangle_classes(
        std::move(__other.triangle_classes)), edges(std::move(__other.edges)) {}

subdivision_table::subdivision_table(std::vector<edge_class> &&__node_positions,
                                     std::vector<subdivision_edge_info> &&__edges) : triangle_classes(
        std::move(__node_positions)), edges(std::move(__edges)) {}

float subdivision_table::class_angle(int __index) {
    return min_angle + step_size * (__index);
}

int subdivision_table::class_index(float __radians) {
    int result = std::floor((__radians - min_angle) / step_size);
    return std::min(std::max(result, 0), step_count - 1);
}

subdivision_table::subdivision_edge_info subdivision_table::edge(int __edge) const { return edges[__edge]; }

coordinate_t
subdivision_table::node_coordinates(edge_id_t __edge, short steiner_index, coordinate_t c1, coordinate_t c2) const {
    const auto info = edge(__edge);

    if (steiner_index == 0)
        return c1;
    if (steiner_index == 1)
        return interpolate_linear(c1, c2, info.r);

    int index = steiner_index - 2 + info.first;
    assert(index >= 0);

    auto last = info.first - 2 + info.node_count;
    std::float16_t relative;
    if (index < last)
        relative = triangle_classes[edge(__edge).edge_class].node_positions[index];
    else
        relative = info.mid_position;

    return interpolate_linear(c1, c2, relative);
}
