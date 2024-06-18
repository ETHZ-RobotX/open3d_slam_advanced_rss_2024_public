#include "RegistrationTestCase.h"

#include <memory>

#include "utils_geometry.h"

RegistrationTestCase::RegistrationTestCase(const std::string name, const PM::DataPoints::Index numberOfPoints, const PM::ScalarType scale,
                                           const PM::StaticCoordVector& trans_origin_ref, const PM::Quaternion& R_origin_ref,
                                           const PM::StaticCoordVector& trans_ref_read, const PM::Quaternion& R_ref_read,
                                           const PM::StaticCoordVector& initialGuess_t_error, const PM::Quaternion& initialGuess_R_error,
                                           const bool useSamePointCloudsForReferenceAndReading)
{
    this->name = name;

    // Shape creation parameters.
    this->numberOfPoints = numberOfPoints;
    this->length *= scale;
    this->width *= scale;
    this->height *= scale;

    // Compute transformations.
    this->T_origin_ref = PM::PointCloudGenerator::buildUpTransformation(trans_origin_ref * scale, R_origin_ref);
    this->T_ref_read = PM::PointCloudGenerator::buildUpTransformation(trans_ref_read * scale, R_ref_read);
    this->T_origin_read = this->T_origin_ref * this->T_ref_read;

    // Compute initial guess.
    this->initialGuessError_T_origin_read = PM::PointCloudGenerator::buildUpTransformation(initialGuess_t_error, initialGuess_R_error);
    this->initialGuess_T_origin_read = this->T_origin_ref * this->initialGuessError_T_origin_read;

    // Generate point clouds.
    // Note: in order to create the situation where the displacement from reading to reference is equal to T_ref_read, we need to
    // transform the reading point cloud by inverse(T_ref_read)
    if (useSamePointCloudsForReferenceAndReading)
    {
        PM::DataPoints pointCloud{ PM::PointCloudGenerator::generateUniformlySampledBox(this->length,
                                                                                        this->width,
                                                                                        this->height,
                                                                                        this->numberOfPoints,
                                                                                        this->identity_T.translation(),
                                                                                        PM::Quaternion(this->identity_T.linear())) };
        std::shared_ptr<PM::Transformation> transformator{ PM::get().REG(PM::Transformation).create("RigidTransformation") };
        this->referencePointCloudInOriginFrame = pointCloud;
        transformator->inPlaceCompute(this->T_origin_ref.matrix(), this->referencePointCloudInOriginFrame);
        this->readingPointCloudInReadingFrame = pointCloud;
        transformator->inPlaceCompute(this->T_ref_read.inverse().matrix(), this->readingPointCloudInReadingFrame);
    }
    else
    {
        this->referencePointCloudInOriginFrame =
            PM::PointCloudGenerator::generateUniformlySampledBox(this->length,
                                                                 this->width,
                                                                 this->height,
                                                                 this->numberOfPoints,
                                                                 this->T_origin_ref.translation(),
                                                                 PM::Quaternion(this->T_origin_ref.linear()));
        this->readingPointCloudInReadingFrame =
            PM::PointCloudGenerator::generateUniformlySampledBox(this->length,
                                                                 this->width,
                                                                 this->height,
                                                                 this->numberOfPoints,
                                                                 this->T_ref_read.inverse().translation(),
                                                                 PM::Quaternion(this->T_ref_read.inverse().linear()));
    }
}

std::ostream& operator<<(std::ostream& ostream, const RegistrationTestCase& testCase)
{
    ostream << "Name: " << testCase.name << "\n";
    ostream << "Transformation from reference point cloud to coordinate frame origin (T_origin_ref):\n"
            << testCase.T_origin_ref.matrix() << "\n";
    ostream << "Transformation from reading point cloud to coordinate frame origin (T_origin_read):\n"
            << testCase.T_origin_read.matrix() << "\n";
    ostream << "Transformation from reading to reference point cloud frame (T_ref_read):\n" << testCase.T_ref_read.matrix() << "\n";
    ostream << "Initial Guess Transformation Error from reading to reference point cloud frame (initialGuessError_T_origin_read):\n"
            << testCase.initialGuessError_T_origin_read.matrix() << "\n";
    ostream << "Initial Guess Transformation from reading point cloud to coordinate frame origin (initialGuess_T_origin_read):\n"
            << testCase.initialGuess_T_origin_read.matrix() << "\n";
    ostream << "Box point clouds with attributes (length = " << testCase.length << ", width = " << testCase.width
            << ", height = " << testCase.height << ", cardinality = " << testCase.numberOfPoints << ")\n";
    return ostream;
}