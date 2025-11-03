#pragma once

#include "geo/include/Point.hpp"

#include <cmath>
#include <numeric>
#include <optional>
#include <vector>

namespace rrt::geo {

template <typename T> class Polygon {
public:
  Polygon() = default;

  Polygon(std::vector<Point<T>> points) : points_{std::move(points)} {}

  const size_t size() const noexcept { return points_.size(); }

  std::optional<Point<T>> gravity_center() const noexcept;

  const std::vector<Point<T>> &points() const noexcept { return points_; }

private:
  std::vector<Point<T>> points_;
};

template <typename T> std::optional<Point<T>> Polygon<T>::gravity_center() const noexcept {
  Point<T> init(T{}, T{});

  if (points_.empty()) {
    return std::nullopt;
  }

  const auto gc =
      std::accumulate(points_.cbegin(), points_.cend(), init, [](const Point<T> &left, const Point<T> &right) {
        return Point<T>(left.x() + right.x(), left.y() + right.y());
      });

  return Point<T>(gc.x() / points_.size(), gc.y() / points_.size());
}

} // namespace rrt::geo
