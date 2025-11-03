#include <geo/include/IncidencePredicates.hpp>
#include <geo/include/Polygon.hpp>
#include <gtest/gtest.h>

TEST(PolygonTest, ConstructPolygon) {

  std::vector<rrt::geo::Point<double>> points = {rrt::geo::Point(2.0, 2.0), rrt::geo::Point(3.0, 2.0),
                                                 rrt::geo::Point(3.0, 3.0), rrt::geo::Point(2.0, 3.0)};

  rrt::geo::Polygon<double> polygon(std::move(points));
  EXPECT_EQ(polygon.size(), 4);
}

TEST(PolygonTest, PointInsideRectangle) {
  using point_t = rrt::geo::Point<double>;
  point_t point0(0.0, 0.0);
  point_t point1(-0.5, -0.5);
  point_t point2(1.0, 1.0);

  std::vector<point_t> poly_points = {{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}};
  rrt::geo::Polygon<double> polygon(std::move(poly_points));

  EXPECT_TRUE(rrt::geo::IncidencePredicates::intersects(point0, polygon.points()));
  EXPECT_TRUE(rrt::geo::IncidencePredicates::intersects(point1, polygon.points()));
  EXPECT_TRUE(rrt::geo::IncidencePredicates::intersects(point2, polygon.points()));
}

TEST(PolygonTest, PointInsideT) {
  using point_t = rrt::geo::Point<double>;
  point_t point0(0.0, 0.5);
  point_t point1(-0.25, 3.25);
  point_t point2(2.0, 1.0);

  // T-shaped polygon
  std::vector<point_t> poly_points = {{0.5, 0.0},  {0.5, 3.0},  {2.0, 3.0},  {2.0, 3.5},
                                      {-2.0, 3.5}, {-2.0, 3.0}, {-0.5, 3.0}, {-0.5, 0.0}};

  rrt::geo::Polygon<double> polygon(std::move(poly_points));

  EXPECT_TRUE(rrt::geo::IncidencePredicates::intersects(point0, polygon.points()));
  EXPECT_TRUE(rrt::geo::IncidencePredicates::intersects(point1, polygon.points()));
  EXPECT_FALSE(rrt::geo::IncidencePredicates::intersects(point2, polygon.points()));
}

TEST(PolygonTest, PointInsideU) {
  using point_t = rrt::geo::Point<double>;
  point_t point0(-4.6, 7.8);
  point_t point1(-0.25, 0.25);
  point_t point2(4.01, 9.99);
  point_t point3(8.01, 8.00);

  // U-shaped polygon
  std::vector<point_t> poly_points = {{5.0, 0.0},  {5.0, 10.0},  {4.0, 10.0},  {4.0, 1.0},
                                      {-4.0, 1.0}, {-4.0, 10.0}, {-5.0, 10.0}, {-5.0, 0.0}};

  rrt::geo::Polygon<double> polygon(std::move(poly_points));

  EXPECT_TRUE(rrt::geo::IncidencePredicates::intersects(point0, polygon.points()));
  EXPECT_TRUE(rrt::geo::IncidencePredicates::intersects(point1, polygon.points()));
  EXPECT_TRUE(rrt::geo::IncidencePredicates::intersects(point2, polygon.points()));
  EXPECT_FALSE(rrt::geo::IncidencePredicates::intersects(point3, polygon.points()));
}
