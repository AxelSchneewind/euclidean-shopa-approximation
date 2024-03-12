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
keep_duplicates_sorted(std::vector<T> &items) {
    size_t j = 0;
    for (size_t i = 0; i < items.size(); ++i) {
        auto element = items[i];

        if ((i == 0 || items[i - 1] == element) && (j == 0 || items[j - 1] != element))
            items[j++] = element;
    }

    items.resize(j);
    items.shrink_to_fit();
}