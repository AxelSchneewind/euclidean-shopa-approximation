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
  std::string directory;
  std::cout << "filename: " << std::flush;
  std::cin >> directory;

  // read triangulation
  std::ifstream input (directory);

  std_graph_t graph (triangulation_file_io::read<std_graph_t> (input));

  // write gl file for graph
  gl_file_io writer;
  std::ofstream output (directory + ".bin");
  writer.write<std_graph_t, stream_encoders::encode_binary>(output, graph);

  output.close();
}