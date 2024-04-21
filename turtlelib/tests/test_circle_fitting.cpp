#include <sstream>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/circle_fitting.hpp"

using turtlelib::normalize_angle;
using turtlelib::normalizeVector;
using turtlelib::Point2D;
using turtlelib::Vector2D;
using turtlelib::PI;
using Catch::Matchers::WithinAbs;
using turtlelib::Circle;
using turtlelib::circle_fitting;

double accuracy = 0.0; // percent

TEST_CASE( "Circle fitting works", "[normalize_angle]" ) 
{
    std::vector<turtlelib::Point2D> cluster1;
    cluster1.push_back(Point2D{1,7});
    cluster1.push_back(Point2D{2,6});
    cluster1.push_back(Point2D{5,8});
    cluster1.push_back(Point2D{9,5});
    cluster1.push_back(Point2D{3,7});

    Circle circle1 = circle_fitting(cluster1);

    accuracy = 6.0; // percent
    REQUIRE_THAT( circle1.x, WithinAbs(4.615482, accuracy * fabs(circle1.x) / 100.0));
    REQUIRE_THAT( circle1.y, WithinAbs(2.807354,accuracy * fabs(circle1.y) / 100.0));
    REQUIRE_THAT( circle1.R, WithinAbs(4.8275,accuracy * fabs(circle1.R) / 100.0));

    std::vector<turtlelib::Point2D> cluster2;
    cluster2.push_back(Point2D{-1,0});
    cluster2.push_back(Point2D{-0.3,-0.06});
    cluster2.push_back(Point2D{0.3,0.1});
    cluster2.push_back(Point2D{1,0});

    Circle circle2 = circle_fitting(cluster2);

    accuracy = 1e-4; // percent
    REQUIRE_THAT( circle2.x, WithinAbs(0.4908357, accuracy * fabs(circle2.x) / 100.0));
    REQUIRE_THAT( circle2.y, WithinAbs(-22.15212,accuracy * fabs(circle2.y) / 100.0));
    REQUIRE_THAT( circle2.R, WithinAbs(22.17979,accuracy * fabs(circle2.R) / 100.0));
}