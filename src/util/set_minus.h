#pragma once

#include <vector>

/**
 * removes the elements of one set from another.
 * expects the spans to be sorted and without duplicates.
 * @tparam T
 * @tparam SetSize
 * @tparam MinusSize
 * @param set
 * @param minus
 * @param out
 * @return
 */
template<typename T, typename Set, typename Minus, typename Out>
size_t set_minus_sorted(const Set &set, const Minus &minus, Out &out) {
    size_t i = 0;
    size_t j = 0;
    size_t k = 0;

    while (i < set.size()) {
        if (i > 0 && set[i] < set[i - 1]) break;

        while (j < minus.size() && set[i] > minus[j])
            j++;

        if (j >= minus.size() || set[i] < minus[j])
            out[k++] = set[i];

        i++;
    }

    return k;
};

template<typename T>
size_t set_minus_sorted(const std::vector<T> &set, const std::vector<T> &minus, std::vector<T> &out) {
    std::span<const T> span1(set.begin(), set.end());
    std::span<const T> span2(minus.begin(), minus.end());
    size_t k = set_minus_sorted<T, std::span<const T>, std::span<const T>, std::vector<T>>(span1, span2, out);
    out.resize(k);
    out.shrink_to_fit();

    return k;
}
