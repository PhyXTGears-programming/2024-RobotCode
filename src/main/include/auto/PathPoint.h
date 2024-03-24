#pragma once

#include "units/velocity.h"
#include "units/angle.h"
#include "units/length.h"

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

enum PathPointType {
    TYPE_OTHER,
    TYPE_HALT
};

class PathPoint {
    public:
        /**
         * Constructs a PathPoint at origin (x, y) facing in 'rotation' with speed of 'velocity'.
         * 
         * @param x The distance downfield
         * @param y The distance to the right of driver TODO: verify
         * @param rotation The rotation direction of the robot at this point
         * @param velocity The speed the robot should be moving at this point
        */
        PathPoint(units::meter_t x, units::meter_t y, frc::Rotation2d rotation, units::meters_per_second_t velocity);

        PathPoint & SetType(const PathPointType type);

        /**
         * Returns a Pose2d with this PathPoint's values.
         * 
         * @return A frc::Pose2d object with this point's x, y, and rotation
        */
        frc::Pose2d Pose() const;

        /**
         * Returns the X component of this PathPoint
         * 
         * @return The X component of this PathPoint
        */
        units::meter_t X() const;
        
        /**
         * Returns the Y component of this PathPoint
         * 
         * @return The Y component of this PathPoint
        */
        units::meter_t Y() const;

        /**
         * Returns the Rotation component of this PathPoint
         * 
         * @return The Rotation component of this PathPoint
        */
        frc::Rotation2d Rotation() const;
        
        /**
         * Returns the Velocity component of this PathPoint
         * 
         * @return The Velocity component of this PathPoint
        */
        units::meters_per_second_t Velocity() const;

        /**
         * TODO: This
        */
        PathPointType Type() const;
    private:
        units::meter_t x;
        units::meter_t y;
        frc::Rotation2d rotation;
        units::meters_per_second_t velocity;
        PathPointType type;
};