#pragma once
#include <concepts>
#include <type_traits>

namespace optional {

    /// template for setting special values that indicate 'missing' value
    /// \tparam T
    template<typename T>
    inline constexpr std::remove_cvref_t<T> none_value;

    /// check if val is 'missing' (i.e. equal to none_value<T>)
    /// \tparam T
    /// \param val
    /// \return
    template<typename T>
    inline constexpr bool is_none(T&& val) { return val == none_value<std::remove_cvref_t<T>>; }
}