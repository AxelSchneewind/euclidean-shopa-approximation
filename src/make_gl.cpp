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

  std::string filename;
  std::string filename_out;
  std::cout << "input filename: " << std::flush;
  std::cin >> filename;
  std::cout << "output filename: " << std::flush;
  std::cin >> filename_out;

  // read graph
  // TODO make dependent on file ending
  std::ifstream input (filename);
  fmi_file_io reader;

  ch_graph_t graph (reader.read<ch_node_t, ch_edge_t> (input));

  // write gl file for graph
  gl_file_io writer;
  std::ofstream output (filename_out);
  writer.write<ch_node_t, ch_edge_t>(output, graph);

  output.close();
}