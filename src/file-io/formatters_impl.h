#pragma once

#include "formatters.h"

#include <iostream>
#include <vector>
#include <span>
#include "../graph/base_types.h"
#include "../graph/unidirectional_adjacency_list.h"

namespace stream_encoders {

std::istream &
encode_text::skip_comments (std::istream &s)
{
  s >> std::ws;
  while (s.peek () == '#' || s.peek () == '\n')
  {
    s.ignore (std::numeric_limits<std::streamsize>::max (), '\n');
    s >> std::ws;
  }
  return s;
}

template <typename T>
T
encode_text::read (std::istream &input)
{
  T result;
  input >> result;
  return result;
}

template <typename T>
std::ostream &
encode_text::write (std::ostream &output, const T &value)
{
  output << value;
  return output;
}


// coordinates
template <>
coordinate_t
encode_text::read (std::istream &input)
{
  coordinate_t result;
  input >> result.latitude >> result.longitude;
  return result;
}

template <>
std::ostream &
encode_text::write (std::ostream &output, const coordinate_t &c)
{
  output << c.latitude << ' ' << c.longitude << ' ';
  return output;
}

// standard nodes and edges
template <>
node_t
encode_text::read (std::istream &input)
{
  node_t result;
  input >> ignore >> ignore >> result.coordinates.latitude >> result.coordinates.longitude >> ignore;
  return result;
}

template <>
std::ostream &
encode_text::write (std::ostream &output, const node_t &node)
{
  output << 0 << ' ' << 0 << ' ' << node.coordinates.latitude << ' ' <<  node.coordinates.longitude << ' ' <<  0;
  return output;
}

template <>
adjacency_list_edge<edge_t>
encode_text::read (std::istream &input)
{
  adjacency_list_edge<edge_t> result;
  input >> result.source >> result.destination >> result.info.cost >> ignore >> ignore;
  return result;
}

template <>
std::ostream &
encode_text::write (std::ostream &output, const adjacency_list_edge<edge_t> &edge)
{
  output << edge.source << ' ' <<  edge.destination << ' ' <<  edge.info.cost << ' ' <<  0 << ' ' <<  0 << ' ';
  return output;
}

// triangles
template <>
triangle
encode_text::read (std::istream &input)
{
  triangle result;
  input >> result[0] >> result[1] >> result[2];
  return result;
}

template <>
std::ostream &
encode_text::write (std::ostream &output, const triangle &triangle)
{
  output << triangle[0] << ' ' <<  triangle[1] << ' ' <<  triangle[2] << ' ';
  return output;
}

// CH nodes and edges
template <>
ch_node_t
encode_text::read (std::istream &input)
{
  ch_node_t result;
  input >> ignore >> ignore >> result.coordinates.latitude >> result.coordinates.longitude >> ignore >> result.level;
  return result;
}

template <>
std::ostream &
encode_text::write (std::ostream &output, const ch_node_t &node)
{
  output << node.coordinates.latitude << ' ' <<  node.coordinates.longitude << ' ' <<  0 << ' ' <<  node.level << ' ';
  return output;
}

template <>
adjacency_list_edge<ch_edge_t>
encode_text::read (std::istream &input)
{
  adjacency_list_edge<ch_edge_t> result;
  input >> result.source >> result.destination >> result.info.cost >> ignore >> ignore >> result.info.edgeA
    >> result.info.edgeB;
  return result;
}

template <>
std::ostream &
encode_text::write (std::ostream &output, const adjacency_list_edge<ch_edge_t> &edge)
{
  output << edge.source << ' ' <<  edge.destination << ' ' <<  edge.info.cost << ' ' <<  0 << ' ' <<  0 << ' ' <<  edge.info.edgeA << ' ' <<  edge.info.edgeB << ' ';
  return output;
}




// read/write lists
template <typename T>
std::vector<T>
encode_text::read (std::istream &input, int count)
{
  std::vector<T> result;
  result.reserve (count);

  for (size_t i = 0; i < count; i++)
    result.emplace_back (read<T> (input));

  return result;
}

template <typename T>
std::ostream &
encode_text::write (std::ostream &output, std::span<const T> &values)
{
  for (size_t i = 0; i < values.extent (); i++)
    write<T> (output, values[i]);
  return output;
}

template <typename T>
T
encode_binary::read (std::istream &input)
{
  T result;
  input.read ((char *) &result, sizeof (T));
  return result;
}

template <typename T>
std::ostream &
encode_binary::write (std::ostream &output, const T &value)
{
  output.write ((char *) &value, sizeof (T));
  return output;
}

template <typename T>
std::vector<T>
encode_binary::read (std::istream &input, int count)
{
  std::vector<T> result (count);
  input.read ((char *) result.data (), sizeof (T) * count);
  return result;
}

template <typename T>
std::ostream &
encode_binary::write (std::ostream &output, std::span<const T> &values)
{
  output.write ((char *) values.begin (), sizeof (T) * values.extent ());
  return output;
}

} // namespace stream_encoders
