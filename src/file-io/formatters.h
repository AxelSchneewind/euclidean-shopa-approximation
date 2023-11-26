#pragma once

#include <iostream>
#include <vector>
#include <span>
#include "../graph/base_types.h"
#include "../graph/unidirectional_adjacency_list.h"

namespace stream_encoders {

    class encode_text {
    public:
        static std::istream &skip_comments(std::istream &s);

        template<typename T>
        static T read(std::istream &input);

        template<typename T>
        static std::ostream &write(std::ostream &output, const T &value);

        // read/write lists
        template<typename T>
        static std::vector<T> read(std::istream &input, int count);

        template<typename T>
        static std::ostream &write(std::ostream &output, std::span<const T> &values);
    };

    class encode_binary {
    public:
        std::istream &skip_comments(std::istream &input) { return input; };

        template<typename T>
        static T read(std::istream &input);

        template<typename T>
        static std::ostream &write(std::ostream &output, const T &value);

        template<typename T>
        static std::vector<T> read(std::istream &input, int count);

        template<typename T>
        static std::ostream &write(std::ostream &output, std::span<const T> &values);
    };

}