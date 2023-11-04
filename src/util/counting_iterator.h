#pragma once
#include <iterator>

template<typename T>
struct counter : public std::iterator<std::forward_iterator_tag, T> {
private:
    T current;
    T max;

    counter(T __current, T __max) : current(__current), max(__max) {}
public:
    counter(T __count) : current(), max(__count) {}

    counter &begin() { return *this; };

    counter end() { return {max, max}; };

    bool operator==(const counter& __other) const { return current == __other.current; }
    bool operator!=(const counter& __other) const { return current != __other.current; }

    counter &operator++(int) {
        current++;
        return *this;
    }
    counter operator++() {
        auto result = *this;
        current++;
        return result;
    }

    const T& operator*() const { return current; }
};
