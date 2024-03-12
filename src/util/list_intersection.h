#pragma once

#include <unordered_map>
#include <vector>

/**
 * intersects two lists
 * @tparam T
 * @param items
 */
template<typename T>
void
list_intersect_sorted(std::vector<T> &left, const std::vector<T> &right) {
    size_t target_index = 0;
    size_t left_index = 0;
    size_t right_index = 0;
    while(left_index < left.size() && right_index < right.size()) {
        auto first = left[left_index];
        auto second = right[right_index];

        if (first < second) {
            left_index++;
        } else if (first > second) {
            right_index++;
        } else if (first == second) {
            if (target_index == 0 || first != left[target_index - 1])
                left[target_index++] = left[left_index++];
            else
                left_index++;
        }
    }

    left.resize(target_index);
    left.shrink_to_fit();
}