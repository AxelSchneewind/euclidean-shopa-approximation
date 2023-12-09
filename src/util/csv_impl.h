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
void format_header(table const &values, std::ostream &out, Columns &&columns) {
    for (auto column: columns)
        out << ',' << column;
    out << '\n' << std::flush;
}

void format_header(table const &values, std::ostream &out) {
    for (auto column: values.columns())
    	out << ',' << column;
    out << '\n' << std::flush;
}


template<typename Columns>
void format_csv(table const &values, std::ostream &out, Columns &&columns) {
    for (std::size_t i = 0; i < values.row_count(); i++) {
    	out << i;
        for (std::size_t j = 0; j < columns.size(); j++)
            out << ',' << values.get(i, j);
        out << '\n';
    }

    out << std::flush;
}

void format_csv(table const &values, std::ostream &out) {
    for (std::size_t i = 0; i < values.row_count(); i++) {
    	out << i;
        for (std::size_t j = 0; j < values.columns().size(); j++)
            out << ',' << values.get(i, j);
        out << '\n';
    }
    out << std::flush;
}

template<typename Columns>
void format_csv_line(table const &values, std::ostream &out, Columns &&columns, int row) {
    	out << row << ',';
        for (std::size_t j = 0; j < columns.size(); j++)
		out << ',' << values.get(row, columns[j]);
    out << '\n' << std::flush;
}

void format_csv_line(table const &values, std::ostream &out, int row) {
   	out << row;
        for (std::size_t j = 0; j < values.columns().size(); j++)
		out << ',' << values.get(row, j);
    out << '\n' << std::flush;
}
