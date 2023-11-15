#include <vector>




template<typename T>
void list_invert(std::vector<T>& list) {
    int size = list.size();
    for (size_t i = 0; i < size / 2; ++i) {
        std::swap(list[i], list[size - i - 1]);
    }
}