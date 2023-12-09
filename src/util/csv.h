#pragma once

#include <map>
#include <string>
#include <iostream>


class table {
private:
    std::vector<std::string> _columns;
    std::vector<std::vector<std::string>> values;

    std::size_t column_index(std::string_view c);

public:
    template<typename Columns>
    table(Columns columns) {
        for (auto &&c: columns) {
            _columns.emplace_back(c);
        }
    }

    template<typename T>
    void put(int column, T &&value);

    std::vector<std::string_view> columns() const;

    std::string_view get(std::size_t row, std::size_t column) const;

    std::string_view get(std::size_t column) const;

    void new_line();

    std::size_t row_count() const;
};


void format_csv(table const &values, std::ostream &out);

template<typename Columns>
void format_csv(table const &values, std::ostream &out, Columns &&columns);
