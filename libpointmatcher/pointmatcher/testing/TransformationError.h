#pragma once

#include <ostream>

#include "../PointMatcher.h"

#include "typedefs.h"

//! A structure used to store the registration error.
struct TransformationError
{
    // Value of the translational and rotational deltas.
    PM::StaticCoordVector deltaTranslation{PM::StaticCoordVector::Zero()};
    Eigen::AngleAxis<PM::ScalarType> deltaRotationAngleAxis{Eigen::AngleAxis<PM::ScalarType>::Identity()};

    //! L2-Norm of the translational error.
    PM::ScalarType translationNorm{ 0 };
    //! Rotation angle, coming from an Angle-axis representation of the orientation error.
    PM::ScalarType rotationAngle{ 0 };

    // Sum operators.
    TransformationError& operator+=(const TransformationError& rhs);
    friend TransformationError operator+(TransformationError lhs, const TransformationError& rhs);

    // Division operators.
    TransformationError& operator/=(const PM::ScalarType scalar);
    friend TransformationError operator/(TransformationError lhs, const PM::ScalarType scalar);
};

//! Ostream operator for writing the values of a TransformationError into a string.
std::ostream& operator<<(std::ostream& ostream, const TransformationError& error);