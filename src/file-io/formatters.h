#pragma once

#include <iostream>
#include <vector>
#include <span>
#include "../graph/base_types.h"
#include "../graph/unidirectional_adjacency_list.h"

namespace stream_encoders {

static long ignore;

class encode_text
{
public:
  std::istream &skip_comments (std::istream &s);

  template <typename T> T read (std::istream &input);

  template <typename T> std::ostream &write (std::ostream &output, const T &value);

  // read/write lists
  template <typename T> std::vector<T> read (std::istream &input, int count);

  template <typename T> std::ostream &write (std::ostream &output, std::span<const T> &values);
};

class encode_binary
{
public:
  std::istream &skip_comments (std::istream &input){};

  template <typename T> T read (std::istream &input);

  template <typename T> std::ostream &write (std::ostream &output, const T &value);

  template <typename T> std::vector<T> read (std::istream &input, int count);

  template <typename T> std::ostream &write (std::ostream &output, std::span<const T> &values);
};

}