#pragma once
#include <iterator>

template<typename T>
struct counter : public std::iterator<std::forward_iterator_tag, T> {
private:
    T current;
    T max;

    counter(T __current, T __max) : current(__current), max(__max) {}
public:
    explicit counter(T __count) : current(), max(__count) {}

    counter &begin() { return *this; };

    counter end() { return {max, max}; };

    bool operator==(counter __other) { return current == __other.current; }
    bool operator!=(counter __other) { return current != __other.current; }

    counter &operator++() {
        current++;
        return *this;
    }

    const T& operator*() const { return current; }
};
