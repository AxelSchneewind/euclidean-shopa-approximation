#include "subdivision_table.h"
#include "subdivision_info.h"

#include "../graph/base_types.h"
#include "../util/is_in_range.h"

#include "fast_map_impl.h"


#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <iostream>


template <bool StoreCoords>
subdivision<StoreCoords>
subdivision<StoreCoords>::make_subdivision_info(const adjacency_list<int> &triangulation,
                                   const std::vector<node_t> &nodes,
                                   const polyhedron<int, 3> &polyhedron,
                                   const std::vector<double> &r_values, double epsilon) {
    size_t edges_capped = 0;

    // store subdivision information here
    std::vector<subdivision_edge_info> result;
    result.resize(triangulation.edge_count());

#pragma omp parallel for
    for (size_t edge_id = 0; edge_id < triangulation.edge_count(); edge_id++) {
        auto node1 = triangulation.source(edge_id);
        auto node2 = triangulation.destination(edge_id);

        // get coordinates
        auto const &c1 = nodes[node1].coordinates;
        auto const &c2 = nodes[node2].coordinates;

        // length |e| of the edge
        double const length = distance(c2, c1);

        // get minimal angle for node1 and node2
        // treat angles > 90 degrees like 90 degrees
        long double angle1 = std::numbers::pi_v<long double> / 2; // between node1->node2 and node1->node3
        long double angle2 = std::numbers::pi_v<long double> / 2; // between node2->node1 and node2->node3

        for (auto &&edge: polyhedron.edges(edge_id)) {
            if (optional::is_none(edge)) continue;

            // make sure node3 is different from node1 and node2
            auto node3 = triangulation.destination(edge);
            if (node3 == node1 || node3 == node2)
                node3 = triangulation.source(edge);
            if (node3 == node1 || node3 == node2) continue;

            auto c3 = nodes[node3].coordinates;

            // compute angles
            auto a1 = inner_angle(c1, c2, c1, c3);
            auto a2 = inner_angle(c2, c1, c2, c3);
            assert(std::fabs((a1 + a2 + inner_angle(c3, c1, c3, c2)) - std::numbers::pi) <
                   std::numbers::pi / 180); // check that sum of inner angles is 180º (+- 1º for rounding)

            if (a1 < angle1)
                angle1 = a1;
            if (a2 < angle2)
                angle2 = a2;
        }
        angle1 = std::clamp(angle1, 0.0l, std::numbers::pi_v<long double> / 2);
        angle2 = std::clamp(angle2, 0.0l, std::numbers::pi_v<long double> / 2);

        // relative value where the mid-point (with max distance to other edges) lies between node1 and node2
        long double const mid_position = 1.0l / (1.0l + (std::sin(angle1) / std::sin(angle2)));
        {
            assert(std::abs(mid_position + (1.0l / (1.0l + std::sin(angle2) / std::sin(angle1))) - 1.0) < 0.01);
            assert(mid_position <= 1 && mid_position >= 0);
        }

        // distance values have been computed already, convert to r(v) relative to this edges length
        long double const factor = epsilon / 5;
        long double r_first = factor * (r_values[node1] / length);
        long double r_second = factor * (r_values[node2] / length);
        if (r_first <= min_r_value || r_second <= min_r_value)
            std::cerr << "a radius on edge " << edge_id << " is below the limit\n";
        r_first = std::clamp(r_first, min_r_value, 1.0l);
        r_second = std::clamp(r_second, min_r_value, 1.0l);

        // the base for computing relative node positions
        long double min_base_first = std::pow(r_first, -1.0 / (max_steiner_count_per_edge / 2));
        long double min_base_second = std::pow(r_second, -1.0 / (max_steiner_count_per_edge / 2));
        assert(r_first * std::pow(min_base_first, max_steiner_count_per_edge) >= 1.0);
        assert(r_second * std::pow(min_base_second, max_steiner_count_per_edge) >= 1.0);
        long double const base_first = std::clamp(1.0l + epsilon * std::sin(angle1), min_base_first, 10.0l);
        long double const base_second = std::clamp(1.0l + epsilon * std::sin(angle2), min_base_second, 10.0l);
        if (base_first == min_base_first || base_second == min_base_second) {
            edges_capped++;
            std::cerr << "number of points on edge " << edge_id << " is bounded by index datatype ("
                      << typeid(steiner_index_type).name() << ")\n";
        }


        // get interval in first half that is between r and mid_value
        size_t left_count;
        {
            // exponential search
            left_count = 1;
            while (left_count < max_steiner_count_per_edge / 4 &&
                   std::pow(base_first, left_count) * r_first < mid_position)
                left_count *= 2;

            // binary search
            size_t max_count = left_count + 1;
            left_count /= 2;
            while (max_count > left_count) {
                if (std::pow(base_first, (max_count + left_count) / 2) * r_first < mid_position)
                    left_count = (max_count + left_count) / 2 + 1;
                else
                    max_count = (max_count + left_count) / 2;
            }

#ifndef NDEBUG
            // sanity check
            assert(left_count >= 0);
            auto relative = std::pow(base_first, (left_count-1)) * r_first;
            assert(left_count == 0 || left_count >= max_steiner_count_per_edge / 4 || relative >= r_first - 0.002F);
            assert(left_count == 0 || left_count >= max_steiner_count_per_edge / 4 || relative <= mid_position + 0.002F);
            relative = std::pow(base_first, (left_count)) * r_first;
            assert(left_count >= max_steiner_count_per_edge / 4 || relative >= mid_position - 0.002F);
#endif
        }

        // get interval in second edge half that is between r and mid_value
        size_t right_count;
        {
            // exponential search
            right_count = 1;
            while (right_count < max_steiner_count_per_edge / 4 &&
                   std::pow(base_second, right_count) * r_second < (1 - mid_position))
                right_count *= 2;

            // binary search
            size_t max_count = right_count + 1;
            right_count /= 2;
            while (max_count > right_count) {
                if (std::pow(base_second, (max_count + right_count) / 2) * r_second < (1 - mid_position))
                    right_count = (max_count + right_count) / 2 + 1;
                else
                    max_count = (max_count + right_count) / 2;
            }

#ifndef NDEBUG
            // sanity check
            assert(right_count >= 0);
            auto relative = std::pow(base_second, (right_count - 1)) * r_second;
            assert(right_count == 0 || right_count >= max_steiner_count_per_edge / 4 || relative >= r_second - 0.002F);
            assert(right_count == 0 || right_count >= max_steiner_count_per_edge / 4 || relative <= (1 - mid_position) + 0.002F);
            relative = std::pow(base_second, (right_count)) * r_second;
            assert(right_count >= max_steiner_count_per_edge / 4 || relative >= (1 - mid_position) - 0.002F);
#endif
        }

        // number of points on first half of edge
        auto mid_index = left_count + 1; // c1, steiner points
        // remove point at r(v) if already over on other half of the edge
        if (left_count > 0 && r_first >= mid_position) {
            mid_index--;
        }
#ifndef NDEBUG
        {
            auto relative = std::pow(base_first, mid_index - 2) * r_first;
            assert(mid_index < 2 || relative >= r_first - 0.002F);
            assert(mid_index < 2 || relative <= mid_position + 0.002F);
        }
#endif

        // number of points (points on first half + mid_node + points on second half + c2
        auto count = 1 + left_count + 1 + right_count + 1;
        // remove point at r(v) if already over on other half of the edge
        if (right_count > 0 && r_second >= 1 - mid_position) {
            count = mid_index + 1;
        }
        assert(count >= 2);

        // check that values are in bounds
        if (!is_in_range(count, 2UL, max_steiner_count_per_edge)
            || !is_in_range(mid_position, 0.0, 1.0)
            || !is_in_range(mid_index, 0UL, count - 1)
            || !is_in_range(r_first, min_r_value, 1.1)
            || !is_in_range(r_second, min_r_value, 1.1)
            || !is_in_range(base_first, min_base_first, std::numeric_limits<float>::max())
            || !is_in_range(base_second, min_base_second, std::numeric_limits<float>::max())
            || !is_in_range(left_count, 0UL, max_steiner_count_per_edge / 2)
            || !is_in_range(right_count, 0UL, max_steiner_count_per_edge / 2))
            throw std::invalid_argument("some value does not fit");

        auto &entry = result[edge_id];
        entry.mid_position = mid_position;
        entry.mid_index = mid_index;
        entry.node_count = count;
        entry.r_first = r_first;
        entry.r_second = r_second;
        entry.base_first = std::log(base_first);
        entry.base_second = std::log(base_second);
    }

    if constexpr (store_node_coordinates) {
        // precompute coordinates
        coordinate_container coordinates(triangulation.edge_count());
        for (edge_id_type edge = 0; edge < triangulation.edge_count(); ++edge) {
            coordinates.append(edge, result[edge].node_count);
            for (int index = 0; index < result[edge].node_count; ++index) {
                coordinates.at(edge, index) = compute_node_coordinates(index, result[edge],
                                                                       nodes[triangulation.source(edge)].coordinates,
                                                                       nodes[triangulation.destination(edge)].coordinates);
            }
        }
	coordinates.shrink_to_fit();

        return {std::move(result), std::move(coordinates)};
    } else {
        return {std::move(result)};
    }
}

template <bool StoreCoords>
subdivision<StoreCoords>::coordinate_type
subdivision<StoreCoords>::compute_node_coordinates(steiner_index_type steiner_index,
                                      subdivision::subdivision_edge_info const &info,
                                      coordinate_t const &c1,
                                      coordinate_t const &c2) {
    assert(steiner_index >= 0);
    assert(steiner_index < info.node_count);

    if (steiner_index == 0)
        return c1;
    if (steiner_index == info.node_count - 1)
        return c2;

    if (steiner_index == info.mid_index)
        return interpolate_linear(c1, c2, info.mid_position);

    if (steiner_index < info.mid_index) [[likely]] {
        auto const index = steiner_index - 1;
        assert(index >= 0);

        auto &&relative = std::exp(info.base_first * index) * info.r_first;
        assert(index != 0 || relative == info.r_first);
        assert(relative >= info.r_first - 0.002F);
        assert(relative <= info.mid_position + 0.002F);
        return interpolate_linear(c1, c2, relative);
    }

    auto const index = info.node_count - steiner_index - 2;
    assert(index >= 0);

    auto &&relative = std::exp(info.base_second * index) * info.r_second;
    assert(steiner_index != info.node_count - 2 || index == 0);
    assert(index != 0 || relative == info.r_second);
    assert(relative >= info.r_second - 0.002F);
    assert(relative <= 1.0F - info.mid_position + 0.002F);
    return interpolate_linear(c2, c1, relative);
}

template <bool StoreCoords>
inline double subdivision<StoreCoords>::relative_position_mid(edge_id_t const edge) const {
    assert(edge >= 0 && static_cast<size_t>(edge) < _edges.size());
    return _edges[edge].mid_position;
}

template <bool StoreCoords>
inline double
subdivision<StoreCoords>::relative_position_steiner(edge_id_t const edge, steiner_index_type const steiner_index) const {
    assert(edge >= 0 && static_cast<size_t>(edge) < _edges.size());
    auto &&info = _edges[edge];

    assert(steiner_index > 0);
    assert(steiner_index < info.node_count - 1);

    if (steiner_index < info.mid_index) [[likely]] {
        auto const index = steiner_index - 1;
        assert(index >= 0);

        auto relative = std::exp(info.base_first * index) * info.r_first;
        assert(index != 0 || relative == info.r_first);
        assert(relative >= info.r_first - 0.002F);
        assert(relative <= info.mid_position + 0.002F);
        [[assume(std::isnormal(relative) && relative >= 0.0)]];
        return std::clamp(relative, 0.0, 1.0);
    }

    auto const index = info.node_count - steiner_index - 2;
    assert(index >= 0);

    auto relative = std::exp(info.base_second * index) * info.r_second;
    assert(steiner_index != info.node_count - 2 || index == 0);
    assert(index != 0 || relative == info.r_second);
    assert(relative >= info.r_second - 0.002F);
    assert(relative <= 1.0F - info.mid_position + 0.002F);
    relative = 1.0 - relative;
    [[assume(std::isnormal(relative) && relative >= 0.0)]];
    return std::clamp(relative, 0.0, 1.0);
}


template <bool StoreCoords>
inline double subdivision<StoreCoords>::relative_position(edge_id_t const edge, steiner_index_type const steiner_index) const {
    assert(edge >= 0 && static_cast<size_t>(edge) < _edges.size());
    auto &&info = _edges[edge];

    assert(steiner_index >= 0);
    assert(steiner_index < info.node_count);

    if (steiner_index == 0)
        return 0.0;
    if (steiner_index == info.node_count - 1)
        return 1.0;

    if (steiner_index == info.mid_index)
        return info.mid_position;

    if (steiner_index < info.mid_index) [[likely]] {
        auto const index = steiner_index - 1;
        assert(index >= 0);

        auto relative = std::exp(info.base_first * index) * info.r_first;
        assert(index != 0 || relative == info.r_first);
        assert(relative >= info.r_first - 0.002F);
        assert(relative <= info.mid_position + 0.002F);
        return relative;
    }

    auto const index = info.node_count - steiner_index - 2;
    assert(index >= 0);

    auto relative = std::exp(info.base_second * index) * info.r_second;
    assert(steiner_index != info.node_count - 2 || index == 0);
    assert(index != 0 || relative == info.r_second);
    assert(relative >= info.r_second - 0.002F);
    assert(relative <= 1.0F - info.mid_position + 0.002F);
    return (1.0 - relative);
}

template <bool StoreCoords>
inline subdivision<StoreCoords>::steiner_index_type subdivision<StoreCoords>::index(const edge_id_t edge, double relative) const {
    assert(edge >= 0 && static_cast<size_t>(edge) < _edges.size());
    auto &&info = _edges[edge];

    assert(relative >= 0 && relative <= 1);

    if (relative < info.mid_position) {
        steiner_index_type const exponent = std::clamp(
                static_cast<steiner_index_type>(std::floor(std::log(relative / info.r_first) / info.base_first)),
                static_cast<steiner_index_type>(0), static_cast<steiner_index_type>(info.mid_index - 1));
        steiner_index_type const index = exponent + 1;
        assert(index > 0 && index <= info.mid_index + 1 && index < info.node_count);
        return index;
    }

    relative = 1 - relative;
    steiner_index_type const exponent = std::clamp(
            static_cast<steiner_index_type>(std::ceil(std::log(relative / info.r_second) / info.base_second)),
            static_cast<steiner_index_type>(0), static_cast<steiner_index_type>(info.node_count - info.mid_index - 1));
    assert(exponent >= 0 && exponent <= info.node_count - 1);
    steiner_index_type const index = (info.node_count - 1) - exponent;
    assert(index >= 0 && index >= info.mid_index - 1 && index < info.node_count);
    return index;
}

template <bool StoreCoords>
inline subdivision<StoreCoords>::subdivision_edge_info &subdivision<StoreCoords>::edge(edge_id_t const edge) {
    assert(edge >= 0 && static_cast<size_t>(edge) < _edges.size());
    return _edges[edge];
}

template <bool StoreCoords>
inline const subdivision<StoreCoords>::subdivision_edge_info &subdivision<StoreCoords>::edge(edge_id_t const edge) const {
    assert(edge >= 0 && static_cast<size_t>(edge) < _edges.size());
    return _edges[edge];
}

template <bool StoreCoords>
std::vector<size_t> subdivision<StoreCoords>::offsets() const {
    std::vector<size_t> results;

    std::size_t index = 0;
    for (auto &&edge_info: _edges) {
        results.push_back(index);
        assert(index + edge_info.node_count >= index);
        index += edge_info.node_count;
    }
    while (results.size() < _edges.size() + 1)
        results.push_back(index);

    return results;
}

template <bool StoreCoords>
subdivision<StoreCoords>::coordinate_type const&
subdivision<StoreCoords>::node_coordinates(edge_id_t edge, subdivision::steiner_index_type steiner_index) const
requires (store_node_coordinates){
    return _coordinates.at(edge, steiner_index);
}

template<bool StoreCoords>
subdivision<StoreCoords>::coordinate_type subdivision<StoreCoords>::node_coordinates(edge_id_t edge,
                                                                             subdivision::steiner_index_type steiner_index,
                                                                             subdivision::coordinate_type const& c1,
                                                                             subdivision::coordinate_type const& c2)
                                                                             const requires(!store_node_coordinates){
    return compute_node_coordinates(steiner_index, _edges[edge], c1, c2);
}
