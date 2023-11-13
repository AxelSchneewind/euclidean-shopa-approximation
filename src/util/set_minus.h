#include <vector>

template <typename T>
void set_minus_sorted(std::vector<T>& set, std::vector<T>& minus) {
    remove_duplicates_sorted(set);
    remove_duplicates_sorted(minus);

    int i = 0;
    int j = 0;
    int k = 0;

    while (i < set.size()) {
        while (j < minus.size() && set[i] > minus[j])
            j++;

        if (j >= minus.size() || set[i] < minus[j])
            set[k++] = set[i];

        i++;
    }

    set.resize(k);
    set.shrink_to_fit();
};