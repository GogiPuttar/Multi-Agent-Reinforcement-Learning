#ifndef TURTLELIB_DIFFDRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.

#include<iosfwd> // contains forward definitions for iostream objects

#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include"turtlelib/diff_drive.hpp"

namespace turtlelib
{

    /// \brief represent a mobile robot's pose
    struct Pose2D
    {
        /// \brief angle with the world frame
        double theta = 0.0;

        /// \brief x position in the world frame
        double x = 0.0;

        /// \brief y position in world frame
        double y = 0.0;
    };

    /// \brief represent a mobile robot's pose
    struct wheelAngles
    {
        /// \brief left wheel angle
        double left = 0.0;

        /// \brief right wheel angle
        double right = 0.0;
    };

    /// \brief models the kinematics of a differential drive robot with a given wheel track and wheel radius
    class DiffDrive
    {

    private:

        /// \brief radius of wheels, in meters
        double wheel_radius;

        /// \brief separation between wheels, in meters
        double wheel_sep;

    public:

        /// \brief angle of rotation of wheels, in radians
        wheelAngles phi;
    
        /// \brief pose of the mobile robot in the world frame
        Pose2D q;

        /// \brief Initialize the kinematics for a new Diff Drive robot
        DiffDrive();

        /// \brief Initialize the kinematics for a general Diff Drive robot
        explicit DiffDrive(double radius, double sep, wheelAngles wheels, Pose2D pose);

        /// \brief Initialize the radius and wheel separation for a diff drive robot
        explicit DiffDrive(double radius, double sep);

        // Drive the robot forward through the wheels, and return the resulting body twist (compute forward velocity kinematics)
        Twist2D driveWheels(wheelAngles delta_phi);

        // Given a body twist, return required wheel angle increment (compute inverse velocity kinematics)
        wheelAngles TwistToWheels(Twist2D V_b);

        // Get wheel radius
        double radius() const;

        // Get wheel separation
        double separation() const;

        // Get wheel angles
        wheelAngles wheels() const;

        // Get pose
        Pose2D pose() const;

        // /// \brief create a transformation that is a pure translation
        // /// \param trans - the vector by which to translate
        // explicit DiffDrive(Vector2D trans);

        // /// \brief create a pure rotation
        // /// \param radians - angle of the rotation, in radians
        // explicit DiffDrive(double radians);

        // /// \brief Create a transformation with a translational and rotational
        // /// component
        // /// \param trans - the translation
        // /// \param radians - the rotation, in radians
        // DiffDrive(Vector2D trans, double radians);

    };
}

#endif
