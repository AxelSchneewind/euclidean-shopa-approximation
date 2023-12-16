#include "subdivision_table.h"

#include "../graph/base_types.h"

#include <vector>


/**
 * checks min <= val < max
 * @tparam Min
 * @tparam Max
 * @tparam Value
 * @return
 */
template<typename Min, typename Max, typename Value>
bool is_between(Value val, Min min, Max max) {
    return min <= val && min < max;
}

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
                                         const std::vector<float> &__r_values, float __epsilon) {

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
            if (node3 == node1 || node3 == node2)
                node3 = __triangulation.source(edge);

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
        float mid_value_second = 1 / (1 + std::sin(angle2) / std::sin(angle1));
        assert(std::abs(mid_value + mid_value_second - 1.0) < 0.001);
        assert(mid_value < 1 && mid_value > 0);

        // store minimal distance from the given mid-point to any other edge, relative to this edges length
        float mid_dist = mid_value * std::sin(angle1);

        // distance values have been computed already, convert to r(v) relative to this edges length
        float r_first = (__epsilon / 5) * __r_values[node1] / length;
        float r_second = (__epsilon / 5) * __r_values[node2] / length;
        assert(r_first >= 0 && r_second >= 0);
        assert(r_first <= 1.0 && r_second < 1.0);

        // get the class this edge belongs to
        auto index = class_index(angle1);
        auto index_second = class_index(angle2);

        // get interval in first half that is between r and mid_value
        auto &node_positions = __table[index].node_positions;
        size_t first_start_index = 0;
        while (first_start_index < node_positions.size() && node_positions[first_start_index] < r_first)
            first_start_index++;

        size_t first_last_index = first_start_index;
        while (first_last_index < node_positions.size() && node_positions[first_last_index] < mid_value)
            first_last_index++;

        assert(first_start_index < first_last_index + 3);
        assert(first_last_index - first_start_index >= 0);

        // get interval in second edge half that is between r and mid_value
        auto &node_positions_second = __table[index_second].node_positions;
        size_t second_start_index = 0;
        while (second_start_index < node_positions_second.size() &&
               node_positions_second[second_start_index] < r_second)
            second_start_index++;

        size_t second_last_index = second_start_index;
        while (second_last_index < node_positions_second.size() &&
               node_positions_second[second_last_index] < (1 - mid_value))
            second_last_index++;
        assert(second_start_index < second_last_index + 3);
        assert(second_last_index - second_start_index >= 0);

        // number of points on first half of edge
        auto mid_index = (first_last_index - first_start_index) + 2;
        // remove point at r(v) if already over on other half of the edge
        if (r_first >= mid_value) {
            r_first = mid_value;
            mid_index--;
        }

        // number of points (points on first half + mid_node + points on second half + c2 + (c2 + r(c2)))
        auto count = mid_index + (second_last_index - second_start_index) + 2 + 1;
        // remove point at r(v) if already over on other half of the edge
        if (r_second >= 1 - mid_value) {
            r_second = 1 - mid_value;
            count--;
        }
        assert(count >= 2);

        // check that values are in bounds
        if (!is_between(count, 2, std::numeric_limits<unsigned short>::max())
            || !is_between(mid_value, 0, 1)
            || !is_between(mid_index, 1, count)
            || !is_between(r_first, 0, 1)
            || !is_between(r_second, 0, 1)
            || !is_between(index, 0, std::numeric_limits<unsigned char>::max())
            || !is_between(index, 0, node_positions.size())
            || !is_between(index_second, 0, std::numeric_limits<unsigned char>::max())
            || !is_between(index_second, 0, node_positions.size())
            || !is_between(first_start_index, 0, std::numeric_limits<unsigned short>::max())
            || !is_between(first_start_index, 0, node_positions.size())
            || !is_between(second_start_index, 0, std::numeric_limits<unsigned short>::max())
            || !is_between(second_start_index, 0, node_positions.size()))
            throw std::invalid_argument("some value does not fit");

        auto entry = subdivision_edge_info{};
        entry.mid_position = static_cast<float>(mid_value);
        entry.mid_index = static_cast<unsigned short>(mid_index);
        entry.node_count = static_cast<unsigned short>(count);
        entry.r_first = r_first;
        entry.r_second = r_second;
        entry.edge_class_first = static_cast<unsigned char>(index);
        entry.edge_class_second = static_cast<unsigned char>(index_second);
        entry.first_start_index = static_cast<unsigned short>(first_start_index);
        entry.second_start_index = static_cast<unsigned short>(second_start_index);
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

int subdivision_table::class_index(double __radians) {
    __radians = std::min(__radians, M_PI_2);
    int result = std::floor((__radians - min_angle) / step_size);
    return std::min(std::max(result, 0), step_count - 1);
}

subdivision_table::subdivision_edge_info subdivision_table::edge(int __edge) const { return edges[__edge]; }

coordinate_t
subdivision_table::node_coordinates(edge_id_t __edge, short steiner_index, coordinate_t c1, coordinate_t c2) const {
    const auto& info = edges[__edge];

    assert(steiner_index >= 0);
    assert(steiner_index < info.node_count);

    if (steiner_index == 0) [[unlikely]]
        return c1;
    if (steiner_index == info.node_count - 1) [[unlikely]]
        return c2;

    if (steiner_index == info.mid_index) [[unlikely]]
        return interpolate_linear(c1, c2, info.mid_position);

    if (steiner_index == 1) [[unlikely]]
        return interpolate_linear(c1, c2, info.r_first);
    if (steiner_index == info.node_count - 2) [[unlikely]]
        return interpolate_linear(c2, c1, info.r_second);


    if (steiner_index < info.mid_index) {
        auto index = steiner_index - 2 + info.first_start_index;
        assert(index >= 0);
        assert(index < triangle_classes[info.edge_class_first].node_positions.size());

        auto relative = triangle_classes[info.edge_class_first].node_positions[index];
        assert(relative >= info.r_first - 0.002F);
        assert(relative <= info.mid_position + 0.002F);
        return interpolate_linear(c1, c2, relative);
    } else {
        auto index = info.node_count - 1 - steiner_index;
        index = index - 2 + info.second_start_index;
        assert(index >= 0);
        assert(index < triangle_classes[info.edge_class_second].node_positions.size());

        auto relative = triangle_classes[info.edge_class_second].node_positions[index];
        assert(relative >= info.r_second - 0.002F);
        assert(relative <= 1.0F - info.mid_position + 0.002F);
        return interpolate_linear(c2, c1, relative);
    }
}
