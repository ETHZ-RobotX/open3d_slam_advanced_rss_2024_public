#pragma once

#include <string>
#include <ostream>

#include "../PointMatcher.h"

#include "typedefs.h"

//! A structure containing the configurations for running a registration test: true transformations, initial guesses,
//! name of the test case, point clouds to register, and their geometric characteristics.
struct RegistrationTestCase
{

    /**
     * @brief Construct a new Registration Test Case object.
     * 
     * @param name  Name of the test case.
     * @param numberOfPoints  Number of points in the testing point clouds.
     * @param scale   Scale factor to 'amplify' the size of the shapes and the distances between them.
     * @param trans_origin_ref  Translation from the reference point cloud to the origin coordinate frame.
     * @param R_origin_ref  Rotation from the reference point cloud to the origin coordinate frame.
     * @param trans_ref_read  Translation from the reading point cloud to the reference point cloud coordinate frame
     * @param R_ref_read  Rotation from the reading point cloud to the reference point cloud coordinate frame.
     * @param initialGuess_t_error  Error intrinsic to the initial guess on the translation from the reading to the origin point cloud coordinate frame.
     * @param initialGuess_R_error  Error intrinsic to the initial guess on the rotation from the reading to the origin point cloud coordinate frame.
     * @param useSamePointCloudsForReferenceAndReading  Whether the same point clouds should represent the reading and reference.
     */
    RegistrationTestCase(const std::string name, const PM::DataPoints::Index numberOfPoints, const PM::ScalarType scale,
                         const PM::StaticCoordVector& trans_origin_ref, const PM::Quaternion& R_origin_ref,
                         const PM::StaticCoordVector& trans_ref_read, const PM::Quaternion& R_ref_read,
                         const PM::StaticCoordVector& initialGuess_t_error, const PM::Quaternion& initialGuess_R_error,
                         const bool useSamePointCloudsForReferenceAndReading);

    //! The name of this test case.
    std::string name;

    //! A-priori known values for transformations.
    //! A reference point cloud can be displaced from the coordinate frame origin, thus we consider three frames:
    //! * Origin,
    //! * Reference,
    //! * Reading frame.
    //! ICP's task is to compute a correction to be applied on the left, for the transformation from reading to origin frame (T_origin_read)
    const PM::AffineTransform identity_T{ PM::AffineTransform::Identity() };
    PM::AffineTransform T_origin_ref{ PM::AffineTransform::Identity() };
    PM::AffineTransform T_origin_read{ PM::AffineTransform::Identity() };
    PM::AffineTransform T_ref_read{ PM::AffineTransform::Identity() };
    PM::AffineTransform initialGuessError_T_origin_read{ PM::AffineTransform::Identity() };
    PM::AffineTransform initialGuess_T_origin_read{ PM::AffineTransform::Identity() };

    //! Point clouds that will be registered.
    PM::DataPoints referencePointCloudInOriginFrame;
    PM::DataPoints readingPointCloudInReadingFrame;

    //! Geometric characteristics of the point clouds that will be registered.
    // Size in number of points.
    PM::DataPoints::Index numberOfPoints{ 500 };
    // Box dimensions.
    PM::ScalarType length{ 1.0 };
    PM::ScalarType width{ 3.0 };
    PM::ScalarType height{ 5.0 };
};

//! Ostream operator for writing the values of a RegistrationTestCase into a string.
std::ostream& operator<<(std::ostream& ostream, const RegistrationTestCase& testCase);