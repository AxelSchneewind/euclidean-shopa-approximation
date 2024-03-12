#pragma once

#include <unordered_map>
#include <vector>

/**
 * removes duplicates in-place
 * @tparam T
 * @param items
 */
template<typename T>
void
remove_duplicates(std::vector<T> &items) {
    std::unordered_map<T, bool, std::hash<T>> found(10000);

    size_t j = 0;
    for (size_t i = 0; i < items.size(); ++i) {
        auto&& element = items[i];

        if (!found.contains(element)) {
            found[element] = true;
            items[j++] = items[i];
        }
    }

    items.resize(j);
    items.shrink_to_fit();
}

/**
 * removes duplicates in-place for sorted lists
 * @tparam T
 * @param items
 */
template<typename T>
void
remove_duplicates_sorted(std::vector<T> &items) {
    size_t j = 1;
    for (size_t i = 1; i < items.size(); ++i) {
        auto const& element = items[i];
        if (element != items[j - 1]) {
            items[j++] = element;
        }
    }

    items.resize(j);
    items.shrink_to_fit();
}