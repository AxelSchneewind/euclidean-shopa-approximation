#pragma once
#include <iterator>

template<typename T>
struct counter {
private:
    T current;
    T max;

public:
    counter(T current, T max) : current(current), max(max) {}
    counter(T count) : current(0), max(count) {}

    counter &begin() { return *this; };

    struct end_type {};
    end_type end() { return end_type{}; };

    bool operator==(counter other) const { return current == other.current; }
    bool operator!=(counter other) const { return current != other.current; }
    bool operator==(end_type /*other*/) const { return current == max; }
    bool operator!=(end_type /*other*/) const { return current != max; }

    counter &operator++(int) {
        current++;
        return *this;
    }
    counter operator++() {
        auto result = *this;
        current++;
        return result;
    }

    T operator*() const { return current; }
};
