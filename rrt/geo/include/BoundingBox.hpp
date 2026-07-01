#pragma once

#include "Point.hpp"

namespace rrt::geo {

template <typename T> class BoundingBox {
public:
  BoundingBox(const Point<T> &first, const Point<T> &second);

  const Point<T> &corner_sw() const;

  const Point<T> &corner_ne() const;

  const T width() const;

  const T height() const;

  void scale(double scale);

  void extend_to_contain_point(const Point<T> &point);

private:
  Point<T> corner_sw_;
  Point<T> corner_ne_;
};

template <typename T> BoundingBox<T>::BoundingBox(const Point<T> &first, const Point<T> &second) {
  corner_sw_ = Point(std::min(first.x(), second.x()), std::min(first.y(), second.y()));
  corner_ne_ = Point(std::max(first.x(), second.x()), std::max(first.y(), second.y()));
}

template <typename T> const Point<T> &BoundingBox<T>::corner_sw() const { return corner_sw_; }

template <typename T> const Point<T> &BoundingBox<T>::corner_ne() const { return corner_ne_; }

template <typename T> const T BoundingBox<T>::width() const { return corner_ne_.x() - corner_sw_.x(); }

template <typename T> const T BoundingBox<T>::height() const { return corner_ne_.y() - corner_sw_.y(); }

template <typename T> void BoundingBox<T>::scale(double scale) {
  const double new_half_width = 0.5 * scale * width();
  const double new_half_height = 0.5 * scale * height();

  const auto center = Point<T>(0.5 * (corner_sw_.x() + corner_ne_.x(), corner_sw_.y() + corner_ne_.y()));

  corner_sw_ = Point<T>(center.x() - new_half_width, center.y() - new_half_height);
  corner_ne_ = Point<T>(center.x() + new_half_width, center.y() + new_half_height);
}

template <typename T> void BoundingBox<T>::extend_to_contain_point(const Point<T> &point) {
  corner_sw_ = Point<T>(std::min(corner_sw_.x(), point.x()), std::min(corner_sw_.y(), point.y()));
  corner_ne_ = Point<T>(std::max(corner_ne_.x(), point.x()), std::max(corner_ne_.y(), point.y()));
}

} // namespace rrt::geo
