
#include "utils_geometry.h"

#include <random>

#include <Eigen/Core>

PM::ScalarType convertRadiansToDegrees(const PM::ScalarType angleRadians)
{
    return angleRadians * (180.0 / M_PI);
}


PM::ScalarType convertDegreesToRadians(const PM::ScalarType angleDegrees)
{
    return angleDegrees * (M_PI / 180.0);
}

// Creates a quaternion given roll, pitch and yaw (in degrees).
PM::Quaternion buildQuaternionFromRPY(PM::ScalarType roll, PM::ScalarType pitch, PM::ScalarType yaw)
{
    PM::Quaternion q{ Eigen::AngleAxisd(convertDegreesToRadians(roll), Eigen::Vector3d::UnitX())
                      * Eigen::AngleAxisd(convertDegreesToRadians(pitch), Eigen::Vector3d::UnitY())
                      * Eigen::AngleAxisd(convertDegreesToRadians(yaw), Eigen::Vector3d::UnitZ()) };
    q.normalize();

    return q;
}

PM::StaticCoordVector buildRandomVectorFromStdDev(PM::ScalarType translationNoiseStdDev)
{
    return PM::StaticCoordVector::Random() * translationNoiseStdDev;
}

PM::Quaternion buildRandomQuaternionFromStdDev(PM::ScalarType rotationNoiseStdDevInRad)
{
    // Generate rotation angle and axis.
    std::random_device randomDevice;
    std::mt19937 randomNumberGenerator(randomDevice());
    std::normal_distribution<PM::ScalarType> randomDistribution(0, rotationNoiseStdDevInRad);
    const PM::ScalarType rotationAngle{ randomDistribution(randomNumberGenerator) };
    const PM::StaticCoordVector rotationAxis{ PM::StaticCoordVector::Random() };

    // Build quaternion
    PM::Quaternion q{ Eigen::AngleAxis<PM::ScalarType>(rotationAngle, rotationAxis.normalized()) };
    q.normalize();

    return q;
}