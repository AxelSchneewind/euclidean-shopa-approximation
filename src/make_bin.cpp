#include <fstream>
#include <string>

#include "file-io/gl_file_io.h"
#include "file-io/gl_file_io_impl.h"
#include "file-io/triangulation_file_io.h"
#include "file-io/triangulation_file_io_impl.h"
#include "routing.h"

int
main (int argc, char const *argv[])
{
  std::string directory = std::string (argv[1]);

  // read triangulation
  std::ifstream input (directory);
  triangulation_file_io reader;

  std_graph_t graph (reader.read<node_t, edge_t> (input));

  // write gl file for graph
  gl_file_io writer;
  std::ofstream output (directory + ".bin");
  writer.write<node_t, edge_t, stream_encoders::encode_binary>(output, graph);

  output.close();
}