#include "TransformationError.h"

#include "utils_geometry.h"

TransformationError& TransformationError::operator+=(const TransformationError& rhs)
{
    this->translationNorm += rhs.translationNorm;
    this->rotationAngle += rhs.rotationAngle;
    return *this;
}

TransformationError operator+(TransformationError lhs, const TransformationError& rhs)
{
    lhs += rhs;
    return lhs;
}

TransformationError& TransformationError::operator/=(const PM::ScalarType scalar)
{
    this->translationNorm /= scalar;
    this->rotationAngle /= scalar;
    return *this;
}

TransformationError operator/(TransformationError lhs, const PM::ScalarType scalar)
{
    lhs /= scalar;
    return lhs;
}

std::ostream& operator<<(std::ostream& ostream, const TransformationError& error)
{
    ostream << "Translation = " << error.translationNorm << " m\n";
    ostream << "Rotation angle = " << convertRadiansToDegrees(error.rotationAngle) << " degrees\n";

    return ostream;
}