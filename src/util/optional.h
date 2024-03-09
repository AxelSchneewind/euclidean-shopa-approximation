#pragma once
#include <concepts>
#include <type_traits>

namespace optional {
    template<typename T>
    inline constexpr std::remove_cvref_t<T> none_value;

    template<typename T>
    inline constexpr bool is_none(T&& val) { return val == none_value<std::remove_cvref_t<T>>; }
}