#pragma once

#include <graaflib/graph.h>
#include <graaflib/io/dot.h>
#include <graaflib/types.h>

#include <fmt/core.h>

#include <filesystem>
#include <string>

class GraphIO {
public:
  /** Export graph and its data to a file for visualization and debugging
   * @param filename name of output file
   **/
  template <typename VertexT, typename EdgeT, typename VertexWriter, typename EdgeWriter>
  static void serialize_to_graphviz(const graaf::undirected_graph<VertexT, EdgeT> &graph, const std::string &filename,
                                    const VertexWriter &vertex_writer, const EdgeWriter &edgeWriter);

  /** Export graph and its data to a file for visualization and debugging
   * @param filename name of output file
   **/
  template <typename VertexT, typename EdgeT, typename VertexWriter>
  static void serialize_to_vtk(const graaf::undirected_graph<VertexT, EdgeT> &graph, const std::string &filename,
                               const VertexWriter &vertex_writer);
};

template <typename VertexT, typename EdgeT, typename VertexWriter, typename EdgeWriter>
void GraphIO::serialize_to_graphviz(const graaf::undirected_graph<VertexT, EdgeT> &graph, const std::string &filename,
                                    const VertexWriter &vertex_writer, const EdgeWriter &edgeWriter) {
  const std::filesystem::path filePath(filename);
  graaf::io::to_dot(graph, filePath, vertex_writer, edgeWriter);
}

template <typename VertexT, typename EdgeT, typename VertexWriter>
void GraphIO::serialize_to_vtk(const graaf::undirected_graph<VertexT, EdgeT> &graph, const std::string &filename,
                               const VertexWriter &vertex_writer) {
  const std::filesystem::path filePath(filename);
  std::ofstream outfile(filename);

  outfile << "<?xml version=\"1.0\"?>\n<VTKFile type=\"PolyData\" "
             "version=\"0.1\" byte_order=\"LittleEndian\">\n";
  outfile << "  <PolyData>\n";

  const auto num_pts = graph.vertex_count();
  const auto num_edges = graph.edge_count();

  const auto piece_header = fmt::format("    <Piece NumberOfPoints=\"{}\" NumberOfLines=\"{}\" "
                                        "NumberOfPolys=\"0\">\n",
                                        num_pts, num_edges);
  outfile << piece_header;
  outfile << "      <!-- Points -->\n";
  outfile << "      <Points>\n";
  outfile << "        <DataArray type=\"Float32\" NumberOfComponents=\"3\" "
             "format=\"ascii\">\n";

  const auto &vertex_map = graph.get_vertices();
  std::vector<graaf::vertex_id_t> vertex_ids;
  vertex_ids.reserve(vertex_map.size());

  for (const auto &[vertex_i, vertex_data] : vertex_map) {
    vertex_ids.push_back(vertex_i);
  }
  std::sort(vertex_ids.begin(), vertex_ids.end());

  for (const auto &vertex_id : vertex_ids) {
    const auto &vertex_data = graph.get_vertex(vertex_id);
    outfile << fmt::format("          {}\n", vertex_writer(vertex_data));
  }

  outfile << "        </DataArray>\n";
  outfile << "      </Points>\n";
  outfile << "      <!-- Lines -->\n";
  outfile << "      <Lines>\n";
  outfile << "        <DataArray type=\"Int32\" Name=\"connectivity\" "
             "format=\"ascii\">\n";

  for (const auto &[vertex_i, vertex_data] : vertex_map) {
    const auto &vertex_neighbours = graph.get_neighbors(vertex_i);
    for (const auto &vertex_j : vertex_neighbours)
      if (vertex_i < vertex_j) {
        outfile << fmt::format("          {} {}\n", vertex_i, vertex_j);
      }
  }
  outfile << "        </DataArray>\n";

  outfile << "        <DataArray type=\"Int32\" Name=\"offsets\" "
             "format=\"ascii\">\n";
  size_t connectivity_offset = 2;
  for (const auto &[vertex_i, vertex_data] : vertex_map) {
    const auto &vertex_neighbours = graph.get_neighbors(vertex_i);
    for (const auto &vertex_j : vertex_neighbours)
      if (vertex_i < vertex_j) {
        outfile << fmt::format("          {}\n", connectivity_offset);
        connectivity_offset += 2;
      }
  }
  outfile << fmt::format("          {}\n", connectivity_offset);

  outfile << "        </DataArray>\n";
  outfile << "      </Lines>\n";
  outfile << "    </Piece>\n";
  outfile << "  </PolyData>\n";
  outfile << "</VTKFile>\n";
  outfile.close();
}