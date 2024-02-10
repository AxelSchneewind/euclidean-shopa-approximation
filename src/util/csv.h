#pragma once

#include <map>
#include <string>
#include <iostream>
#include <vector>


class table {
private:
    std::vector<std::string> _columns;
    std::vector<std::vector<std::string>> values;

    std::size_t column_index(std::string_view c);

public:
    table() = default;

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

template<typename Columns>
void format_header(table const &values, std::ostream &out, Columns &&columns);

void format_header(table const& values, std::ostream &out); 

void format_csv_line(table const &values, std::ostream &out, int row);

template<typename Columns>
void format_csv_line(table const &values, std::ostream &out, Columns &&columns, int row);

void format_csv(table const &values, std::ostream &out);

template<typename Columns>
void format_csv(table const &values, std::ostream &out, Columns &&columns);
