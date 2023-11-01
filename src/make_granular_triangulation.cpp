#include "routing.h"

#include "file-io/triangulation_file_io.h"
#include "file-io/gl_file_io.h"

#include "triangulation/steiner_graph.h"
#include <fstream>

int main(int argc, const char** argv) {
  std::ifstream input(std::string(argv[1]) + "/aegaeis/toy.graph");
    std::ofstream output(std::string(argv[1]) + "/aegaeis/toy_steiner.gl");

  triangulation_file_io t;
  auto graph = t.template read<steiner_graph>(input);

  gl_file_io out;
  out.write(output, graph);


  return 0;
}