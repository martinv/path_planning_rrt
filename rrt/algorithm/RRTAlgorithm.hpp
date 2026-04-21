#pragma once

#include <array>
#include <random>

#include <graaflib/graph.h>

#include "geo/include/Point.hpp"
#include "geo/include/Polygon.hpp"

namespace rrt::algorithm {

template <typename T> class RRTAlgorithm {
public:
  RRTAlgorithm();

  void construct_rrt_tree(const rrt::geo::Point<T> &start, const rrt::geo::Point<T> &destination,
                          const std::vector<geo::Polygon<T>> &obstacles,
                          graaf::undirected_graph<typename rrt::geo::Point<T>, T> &graph) {}

private:
  std::mt19937 m_randomGen;
  std::uniform_real_distribution<> m_dist;
};

template <typename T> RRTAlgorithm<T>::RRTAlgorithm() {
  std::random_device rd;
  // m_randomGen = rd();
  std::mt19937 gen(rd());
  m_randomGen = std::move(gen);

  m_dist = std::uniform_real_distribution<>(0.0, 1.0);
}

} // namespace rrt::algorithm
