#pragma once

#include <cmath>

#include "../PointMatcher.h"
#include "typedefs.h"

//! Converts an angle from radians to degrees.
PM::ScalarType convertRadiansToDegrees(const PM::ScalarType angleRadians);

//! Converts an angle from degrees to radians.
PM::ScalarType convertDegreesToRadians(const PM::ScalarType angleDegrees);

//! Creates a quaternion given roll, pitch and yaw (in degrees).
//! The quaternion is created through an XYZ Euler Angles parameterization that receives the RPY values.
PM::Quaternion buildQuaternionFromRPY(PM::ScalarType roll, PM::ScalarType pitch, PM::ScalarType yaw);

//! Creates a random vector with a given standard deviation in its coordinates.
//! The output vector has random components, with a magnitude equal to the standard deviation.
PM::StaticCoordVector buildRandomVectorFromStdDev(PM::ScalarType translationNoiseStdDev);

//! Creates a quaternion from a single standard deviation factor that defines the span of
//! a distribution of rotation angles. We sample an angle from that distribution
//! to form a quaternion an Angle-Axis parameterization around a randomly-sampled axis.
PM::Quaternion buildRandomQuaternionFromStdDev(PM::ScalarType rotationNoiseStdDevInDeg);