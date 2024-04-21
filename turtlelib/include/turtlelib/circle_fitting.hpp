#ifndef CIRCLE_FITTING_INCLUDE_GUARD_HPP
#define CIRCLE_FITTING_INCLUDE_GUARD_HPP
/// \file
/// \brief Circle fitting algorithm
// https://nu-msr.github.io/navigation_site/lectures/circle_fit.html
// https://projecteuclid.org/journals/electronic-journal-of-statistics/volume-3/issue-none/Error-analysis-for-circle-fitting-algorithms/10.1214/09-EJS419.full

#include <iosfwd>
#include <cmath>
#include <armadillo>
#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
    /// \brief Circle size and location
    struct Circle
    {
        /// \brief the x coordinate of the circle's centroid
        double x = 0.0;

        /// \brief the y coordinate of the circle's centroid
        double y = 0.0;

        /// \brief the radius of the circle
        double R = 0.0;
    };

    /// \brief circle fitting algorithm
    /// \param cluster (std::vector<turtlelib::Vector2D>) pass one cluster in to fit a circle to
    /// \return radius and x,y coordinates of circle (turtlelib::Circle)
    Circle circle_fitting(std::vector<turtlelib::Point2D> cluster);

}

#endif