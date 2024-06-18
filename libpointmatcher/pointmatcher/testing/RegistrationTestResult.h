#pragma once

#include <ostream>

#include "../PointMatcher.h"

#include "typedefs.h"

//! A struct for storing the results of a registration test, containing algorithmic outputs, such as transformations, error messages, success flags, and metrics.
struct RegistrationTestResult
{
    //! Computed values.
    PM::AffineTransform icpCorrection_T_origin_read{ PM::AffineTransform::Identity() };
    PM::AffineTransform icp_T_origin_read{ PM::AffineTransform::Identity() };

    //! Transformed point clouds.
    // The reading point cloud transformed to the origin frame, according to the initial guess of T_origin_read.
    PM::DataPoints readingPointCloudInOriginFrameFollowingInitialGuess;
    // The reading point cloud transformed to the origin frame, according to the ICP-corrected T_origin_read.
    PM::DataPoints readingPointCloudInOriginFrameFollowingIcpCorrection;

    //! Registration metrics
    // Messages written to the logs by ICP.
    std::string message;
    // Whether ICP successfully converged.
    bool success{ false };
    // Overlap between the reference and reading point cloud [0,1].
    PM::ScalarType pointCloudOverlap{ 0.0 };
    // Whether maximum number of iterations were reached.
    bool isMaxNumberOfIterationsReached{ false };
};

//! Ostream operator for writing the values of a RegistrationTestResult into a string.
std::ostream& operator<<(std::ostream& ostream, const RegistrationTestResult& testResult);
