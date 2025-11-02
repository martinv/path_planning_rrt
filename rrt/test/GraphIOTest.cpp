#include "utils/include/GraphIO.hpp"
#include "geo/include/Point.hpp"
#include <gtest/gtest.h>

TEST(GraphIOTest, WriteGraphToGraphviz) {

  using VertexT = rrt::geo::Point<double>;
  using EdgeT = double;

  const auto graphviz_vertex_writer{
      [](graaf::vertex_id_t vertex_id, const VertexT &vertex) -> std::string {
        std::string color = "mediumspringgreen";
        return fmt::format(
            "label=\"{} [{:.3f},{:.3f}]\", fillcolor={}, style=filled",
            vertex_id, vertex.x(), vertex.y(), color);
      }};

  const auto graphviz_edge_writer{[](const graaf::edge_id_t & /*edge_id*/,
                                     const auto &edge) -> std::string {
    const std::string style{"solid"};
    return fmt::format("label=\"{}\", style={}, color=gray, fontcolor=gray",
                       edge, style);
  }};

  graaf::undirected_graph<VertexT, EdgeT> graph;
  const auto v0 = graph.add_vertex(VertexT(-4.0, -1.0));
  const auto v1 = graph.add_vertex(VertexT(4.0, -1.0));
  const auto v2 = graph.add_vertex(VertexT(4.0, 1.0));
  const auto v3 = graph.add_vertex(VertexT(-4.0, 1.0));

  graph.add_edge(v0, v1, 1.1);
  graph.add_edge(v1, v2, 2.1);
  graph.add_edge(v2, v3, 3.1);
  graph.add_edge(v3, v0, 4.1);

  GraphIO graph_io;
  graph_io.serialize_to_graphviz(graph, "test_graph.dot",
                                 graphviz_vertex_writer, graphviz_edge_writer);
}

TEST(GraphIOTest, WriteGraphToVtk) {

  using VertexT = rrt::geo::Point<double>;
  using EdgeT = double;

  const auto vtk_vertex_writer{[](const VertexT &vertex) -> std::string {
    return fmt::format("{:.6f} {:.6f} {:.6f}", vertex.x(), vertex.y(), 0.0);
  }};

  graaf::undirected_graph<VertexT, EdgeT> graph;
  const auto v0 = graph.add_vertex(VertexT(-4.0, -1.0));
  const auto v1 = graph.add_vertex(VertexT(4.0, -1.0));
  const auto v2 = graph.add_vertex(VertexT(4.0, 1.0));
  const auto v3 = graph.add_vertex(VertexT(-4.0, 1.0));

  graph.add_edge(v0, v1, 1.1);
  graph.add_edge(v1, v2, 2.1);
  graph.add_edge(v2, v3, 3.1);
  graph.add_edge(v3, v0, 4.1);

  GraphIO graph_io;
  graph_io.serialize_to_vtk(graph, "test_graph.vtp", vtk_vertex_writer);
}
