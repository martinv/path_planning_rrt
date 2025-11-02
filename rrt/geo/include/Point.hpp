#pragma once

#include <array>

namespace rrt::geo {

template <typename T> class Point {
public:
  Point() = default;

  Point(const T &x, const T &y);

  Point(const Point &other_point) = default;

  T x() const noexcept { return coord_[0]; }

  T y() const noexcept { return coord_[1]; }

private:
  std::array<T, 2> coord_;
};

template <typename T> Point<T>::Point(const T &x, const T &y) {
  coord_[0] = x;
  coord_[1] = y;
}

} // namespace rrt::geo
