#include <fmt/core.h>
#include <graaflib/graph.h>
// #include <graaflib/io/dot.h>
// #include <graaflib/types.h>

#include "algorithm/RRTAlgorithm.hpp"
#include "geo/include/Polygon.hpp"

#include <filesystem>
#include <iostream>
#include <string>

int main() {

  rrt::geo::Point<double> point(2.0, 3.0);

  std::vector<rrt::geo::Point<double>> points = {rrt::geo::Point(2.0, 2.0), rrt::geo::Point(3.0, 2.0),
                                                 rrt::geo::Point(3.8, 2.6), rrt::geo::Point(3.0, 3.0),
                                                 rrt::geo::Point(2.4, 2.8), rrt::geo::Point(2.0, 3.0)};

  rrt::geo::Polygon<double> polygon_01(std::move(points));

  rrt::geo::Point<double> start(0.0, 0.0);
  rrt::geo::Point<double> destination(5.0, 4.0);
  graaf::undirected_graph<typename rrt::geo::Point<double>, double> rrt_tree;

  rrt::algorithm::RRTAlgorithm<double> rrt_algorithm;
  rrt_algorithm.construct_rrt_tree(start, destination, {polygon_01}, rrt_tree);

  const rrt::geo::Point<double> gravity_center = polygon_01.gravity_center().value();
  std::cout << "Gravity center = [" << gravity_center.x() << ", " << gravity_center.y() << "]\n";

  return 0;
}
