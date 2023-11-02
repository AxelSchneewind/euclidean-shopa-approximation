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

  ch_graph_t graph (fmi_file_io::read<ch_graph_t> (input));

  // write gl file for graph
  std::ofstream output (filename_out);
  gl_file_io::write<ch_graph_t>(output, graph);

  output.close();
}