#pragma once

#include "csv.h"

std::vector<std::string_view> table::columns() const {
    std::vector<std::string_view> result;
    for (auto &&c: _columns)
        result.emplace_back(c);

    return result;
}


void table::new_line() {
    values.emplace_back(_columns.size(), "");
}

std::size_t table::row_count() const { return values.size(); }

std::size_t table::column_index(std::string_view c) {
    for (int i = 0; i < _columns.size(); ++i) {
        if (_columns[i] == c)
            return i;
    }
}

std::string_view table::get(std::size_t row, std::size_t column) const {
    return values[row][column];
}

std::string_view table::get(std::size_t column) const {
    return values.back()[column];
}

template<typename T>
void table::put(int column, T &&value) {
    values.back()[column] = std::format("{}", value);
}

template<typename Columns>
void format_csv(table const &values, std::ostream &out, Columns &&columns) {
    for (auto column: columns)
        out << ',' << column;
    out << '\n';

    for (std::size_t i = 0; i < values.row_count(); i++) {
    	out << i;
        for (std::size_t j = 0; j < columns.size(); j++)
            out << ',' << values.get(i, j);
        out << '\n';
    }

    out << std::flush;
}

void format_csv(table const &values, std::ostream &out) {
    format_csv(values, out, values.columns());
}
