#include <concepts>

namespace optional {
    template<typename T>
    constexpr std::remove_cvref_t<T> none_value;

    template<typename T>
    constexpr bool is_none(T val) { return val == none_value<T>; }
}