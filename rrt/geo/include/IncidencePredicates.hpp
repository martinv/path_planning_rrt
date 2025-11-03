#pragma once

#include "Point.hpp"

#include <cmath>
#include <vector>

namespace rrt::geo {

class IncidencePredicates {
public:
  template <typename T> static bool intersects(const Point<T> &point, const std::vector<Point<T>> &polygon_points);
};

template <typename T>
bool IncidencePredicates::intersects(const Point<T> &point, const std::vector<Point<T>> &polygon_points) {
  // Check if a point lies on a single polygon edge
  auto point_on_edge = [](const Point<T> &seg_start, const Point<T> &seg_end, const Point<T> &point,
                          double eps = 1.e-9) -> bool {
    // Compute cross product of AP x AB. If the cross product is large, then AP
    // and AB are not collinear and P can't lie on the line given by AB
    const double cross = (point.y() - seg_start.y()) * (seg_end.x() - seg_start.x()) -
                         (point.x() - seg_start.x()) * (seg_end.y() - seg_start.y());
    if (std::abs(cross) > eps) {
      return false;
    }
    // Check if P lies within A–B segment bounds
    const double dot = (point.x() - seg_start.x()) * (seg_end.x() - seg_start.x()) +
                       (point.y() - seg_start.y()) * (seg_end.y() - seg_start.y());
    if (dot < 0.0) {
      return false;
    }
    const double len_sq = (seg_end.x() - seg_start.x()) * (seg_end.x() - seg_start.x()) +
                          (seg_end.y() - seg_start.y()) * (seg_end.y() - seg_start.y());
    if (dot > len_sq) {
      return false;
    }
    return true;
  };

  // Segment is given by 2 points: seg_start, seg_end
  // The task is to compute the intersection of the segment with a ray vector
  // starting from point We are trying to solve point + t * ray = seg_start + u *
  // (seg_end - seg_start), i.e. [ray (seg_end - SegStart)] * [t, u]^T = seg_start -
  // point for t and u

  const Point<T> ray{1.0, 0.0};

  auto ray_intersects_segment = [](const Point<T> &ray_origin, const Point<T> &ray_dir, const Point<T> &seg_start,
                                   const Point<T> &seg_end) -> bool {
    // Create a vector connecting end points of segment
    const Point<T> s(seg_end.x() - seg_start.x(), seg_end.y() - seg_start.y());

    // Determinant of linear system, if close to zero, matrix is singular
    // (ray is parallel to segment)
    const double det = ray_dir.x() * s.y() - ray_dir.y() * s.x();
    if (std::abs(det) < 1e-8) {
      return false;
    }

    const Point<T> rhs(seg_start.x() - ray_origin.x(), seg_start.y() - ray_origin.y());

    const double t = (rhs.x() * s.y() - rhs.y() * s.x()) / det;
    const double u = (rhs.x() * ray_dir.y() - rhs.y() * ray_dir.x()) / det;
    if (t >= 0.0 && u >= 0.0 && u <= 1.0) {
      return true;
    }
    return false;
  };

  const auto n_polygon_points = polygon_points.size();

  // First stage: check of point is on polygon boundary
  for (int id_start = 0; id_start < n_polygon_points; ++id_start) {
    const int id_end = (id_start + 1) % n_polygon_points;
    if (point_on_edge(polygon_points[id_start], polygon_points[id_end], point)) {
      return true;
    }
  }

  // Second stage: check if point is in polygon interior
  int num_intersections = 0;
  for (int id_start = 0; id_start < n_polygon_points; ++id_start) {
    const int id_end = (id_start + 1) % n_polygon_points;
    if (ray_intersects_segment(point, ray, polygon_points[id_start], polygon_points[id_end])) {
      num_intersections++;
    }
  }
  return (num_intersections % 2 == 1);
}

} // namespace rrt::geo