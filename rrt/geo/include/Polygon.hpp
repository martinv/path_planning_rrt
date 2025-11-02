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

template <typename T>
std::optional<Point<T>> Polygon<T>::gravity_center() const noexcept {
  Point<T> init(T{}, T{});

  if (points_.empty()) {
    return std::nullopt;
  }

  const auto gc = std::accumulate(
      points_.cbegin(), points_.cend(), init,
      [](const Point<T> &left, const Point<T> &right) {
        return Point<T>(left.x() + right.x(), left.y() + right.y());
      });

  return Point<T>(gc.x() / points_.size(), gc.y() / points_.size());
}

template <typename T>
bool point_is_inside(const Polygon<T> &polygon, const Point<T> &point) {
  // Segment is given by 2 points: seg_start, seg_end
  // The task is to compute the intersection of the segment with a ray vector
  // starting from point We are trying to solve point + t * ray = seg_start + u
  // * (seg_end - seg_start), i.e. [ray (seg_end - seg_start)] * [t, u]^T =
  // seg_start - point for t and u

  const auto &points = polygon.points();

  const Point<T> ray(1.0, 0.0);

  auto ray_intersects_segment =
      [](const Point<T> &ray_origin, const Point<T> &ray_dir,
         const Point<T> &seg_start, const Point<T> &seg_end) -> bool {
    // Create a vector connecting end points of segment
    const Point<T> s(seg_end.x() - seg_start.x(), seg_end.y() - seg_start.y());

    // Determinant of linear system, if close to zero, matrix is singular
    // (ray is parallel to segment)
    const double det = ray_dir.x() * s.y() - ray_dir.y() * s.x();
    if (std::abs(det) < 1e-8) {
      return false;
    }

    const Point<T> rhs(seg_start.x() - ray_origin.x(),
                       seg_start.y() - ray_origin.y());

    const double t = (rhs.x() * s.y() - rhs.y() * s.x()) / det;
    const double u = (rhs.x() * ray_dir.y() - rhs.y() * ray_dir.x()) / det;
    if (t >= 0.0 && u >= 0.0 && u <= 1.0) {
      return true;
    }
    return false;
  };

  int num_intersections = 0;
  for (int id_start = 0; id_start < points.size(); ++id_start) {
    const int id_end = (id_start + 1) % points.size();
    if (ray_intersects_segment(point, ray, points[id_start], points[id_end])) {
      num_intersections++;
    }
  }

  return (num_intersections % 2 == 1);
}

} // namespace rrt::geo
