#include <fmt/core.h>

#include "geo/include/Polygon.hpp"

#include <filesystem>
#include <iostream>
#include <string>

int main() {

  rrt::geo::Point<double> point(2.0, 3.0);

  std::vector<rrt::geo::Point<double>> points = {
      rrt::geo::Point(2.0, 2.0), rrt::geo::Point(3.0, 2.0),
      rrt::geo::Point(3.0, 3.0), rrt::geo::Point(2.0, 3.0)};

  rrt::geo::Polygon<double> polygon(std::move(points));

  const rrt::geo::Point<double> gravity_center =
      polygon.gravity_center().value();
  std::cout << "Gravity center = [" << gravity_center.x() << ", "
            << gravity_center.y() << "]\n";

  return 0;
}
