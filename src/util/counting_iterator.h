#pragma once
#include <iterator>

template<typename T>
struct counter {
private:
    T current;
    T max;

public:
    counter(T __current, T __max) : current(__current), max(__max) {}
    counter(T __count) : current(0), max(__count) {}

    counter &begin() { return *this; };

    struct end_type {};
    end_type end() { return end_type{}; };

    bool operator==(counter __other) const { return current == __other.current; }
    bool operator!=(counter __other) const { return current != __other.current; }
    bool operator==(end_type __other) const { return current == max; }
    bool operator!=(end_type __other) const { return current != max; }

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
