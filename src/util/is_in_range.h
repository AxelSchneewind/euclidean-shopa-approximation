#pragma once

/**
 * checks min <= val < max
 * @tparam Min
 * @tparam Max
 * @tparam Value
 * @return
 */
template<typename Min, typename Max, typename Value>
static bool is_in_range(Value val, Min min, Max max) {
    return val >= min && val < max;
}