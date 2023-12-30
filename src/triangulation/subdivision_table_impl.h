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
subdivision_table::min_r_per_triangle_class(const std::vector<node_t> &nodes, const std::vector<float> &r_values,
                                            const std::vector<triangle> &faces) {
    std::vector<float> min_r(M_PI / step_size, infinity<float>);

    for (auto face: faces) {
        for (int node_index = 0; node_index < 3; ++node_index) {
            auto c1 = nodes[face[node_index]].coordinates;
            auto c2 = nodes[face[(node_index + 1) % 3]].coordinates;
            auto c3 = nodes[face[(node_index + 2) % 3]].coordinates;
            double radians = inner_angle(c1, c2, c1, c3);
            int index = class_index(radians);

            double r_relative = r_values[face[node_index]] / distance(c2, c1);
            min_r[index] = std::min(min_r[index], (float)r_relative);
        }
    }

    return min_r;
}

std::vector<subdivision_table::edge_class>
subdivision_table::precompute(double epsilon, double min_relative_r_value) {
    std::vector<edge_class> triangle_classes;

    for (int index = 0; index < step_count; ++index) {
        double angle = class_angle(index);
        assert(class_index(angle) <= index);
        double sine = std::sin(angle);

        triangle_classes.emplace_back();

        double relative = min_relative_r_value;
        double edge_distance = 0.0;

        while (relative < 1.0) {
            triangle_classes.back().node_positions.emplace_back(relative);
            edge_distance = sine * relative;
            relative += epsilon * edge_distance;
        }
    }

    return triangle_classes;
}

std::vector<subdivision_table::subdivision_edge_info>
subdivision_table::make_subdivision_info(const adjacency_list<int, std::nullptr_t> &triangulation,
                                         const std::vector<node_t> &nodes,
                                         const polyhedron<adjacency_list<int, std::nullptr_t>, 3> &polyhedron,
                                         const std::vector<edge_class> &table,
                                         const std::vector<double> &r_values,
                                         double epsilon) {

    // store subdivision information here
    std::vector<subdivision_edge_info> result;
    result.reserve(triangulation.edge_count());

    for (size_t i = 0; i < triangulation.edge_count(); i++) {
        auto node1 = triangulation.source(i);
        auto node2 = triangulation.destination(i);
        auto inv_edge = triangulation.edge_id(node2, node1);

        // get coordinates
        auto c1 = nodes[node1].coordinates;
        auto c2 = nodes[node2].coordinates;

        // get minimal angle for node1 and node2
        // treat angles > 90 degrees like 90 degrees
        double angle1 = M_PI_2; // between node1->node2 and node1->node3
        double angle2 = M_PI_2; // between node2->node1 and node2->node3
        double angle3 = 0; // between node2->node1 and node2->node3

        for (auto edge: polyhedron.edges(i)) {
            if (is_none(edge)) continue;

            // make sure node3 is different from node1 and node2
            auto node3 = triangulation.destination(edge);
            if (node3 == node1 || node3 == node2)
                node3 = triangulation.source(edge);
            if (node3 == node1 || node3 == node2) continue;

            auto c3 = nodes[node3].coordinates;

            // compute angles
            auto a1 = inner_angle(c1, c2, c1, c3);
            auto a2 = inner_angle(c2, c1, c2, c3);
            auto a3 = inner_angle(c3, c1, c3, c2);
            assert(std::fabs((a1 + a2 + a3) - M_PI) < M_PI / 180); // check that sum of inner angles is 180º (+- 1º for rounding)

            if (a1 < angle1)
                angle1 = a1;
            if (a2 < angle2)
                angle2 = a2;
        }
        angle1 = std::max(angle1, min_angle);
        angle2 = std::max(angle2, min_angle);
        angle1 = std::min(angle1, max_angle);
        angle2 = std::min(angle2, max_angle);
        angle3 = M_PI - angle2 - angle1;

        // length |e| of the edge
        double length = distance(c2, c1);

        // relative value where the mid-point (with max distance to other edges) lies between node1 and node2
        double mid_value = 1 / (1 + std::sin(angle1) / std::sin(angle2));
        double mid_value_second = 1 / (1 + std::sin(angle2) / std::sin(angle1));
        assert(std::abs(mid_value + mid_value_second - 1.0) < 0.001);
        assert(mid_value < 1 && mid_value > 0);

        // distance values have been computed already, convert to r(v) relative to this edges length
        double factor = std::min(epsilon, 1.0) / 5;
        double r_first = factor * (r_values[node1] / length);
        double r_second = factor * (r_values[node2] / length);
        assert(r_first >= 0 && r_second >= 0);
        assert(r_first <= 1.0 && r_second <= 1.0);

        // get the class this edge belongs to
        auto index = class_index(angle1);
        auto index_second = class_index(angle2);

        // get interval in first half that is between r and mid_value
        auto const& node_positions = table[index].node_positions;
        size_t first_start_index = 0;
        while (first_start_index < node_positions.size() && node_positions[first_start_index] < r_first)
            first_start_index++;

        size_t first_last_index = first_start_index;
        while (first_last_index < node_positions.size() && node_positions[first_last_index] < mid_value)
            first_last_index++;

        assert(first_start_index < first_last_index + 3);
        assert(first_last_index - first_start_index >= 0);

        // get interval in second edge half that is between r and mid_value
        auto const&node_positions_second = table[index_second].node_positions;
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
        entry.r_first = static_cast<float>(r_first);
        entry.r_second = static_cast<float>(r_second);
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

subdivision_table::subdivision_table(std::vector<edge_class> &&__node_positions, std::vector<subdivision_edge_info> &&__edges)
: triangle_classes( std::move(__node_positions))
, edges(std::move(__edges)) {}

double subdivision_table::class_angle(int index) {
    return min_angle + (step_size * index);
}

int subdivision_table::class_index(double radians) {
    assert(radians >= 0);
    radians = std::min(radians, M_PI_2);
    int result = std::floor((radians - min_angle) / step_size);
    return std::min(std::max(result, 0), step_count - 1);
}

subdivision_table::subdivision_edge_info const& subdivision_table::edge(int edge) const { return edges[edge]; }
subdivision_table::subdivision_edge_info & subdivision_table::edge(int edge) { return edges[edge]; }

coordinate_t
subdivision_table::node_coordinates(edge_id_t edge, short steiner_index, coordinate_t const& c1, coordinate_t const& c2) const {
    const auto& info = edges[edge];

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

        auto&& relative = triangle_classes[info.edge_class_first].node_positions[index];
        assert(relative >= info.r_first - 0.002F);
        assert(relative <= info.mid_position + 0.002F);
        return interpolate_linear(c1, c2, relative);
    } else {
        auto index = info.node_count - 1 - steiner_index;
        index = index - 2 + info.second_start_index;
        assert(index >= 0);
        assert(index < triangle_classes[info.edge_class_second].node_positions.size());

        auto&& relative = triangle_classes[info.edge_class_second].node_positions[index];
        assert(relative >= info.r_second - 0.002F);
        assert(relative <= 1.0F - info.mid_position + 0.002F);
        return interpolate_linear(c2, c1, relative);
    }
}
