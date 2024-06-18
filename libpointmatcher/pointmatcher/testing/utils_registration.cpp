
#include "utils_registration.h"

#include <fstream>
#include <memory>

#include "utils_filesystem.h"


bool configureIcp(const std::string& filePath, const std::string& configFileName, PM::ICP& icp)
{
    const std::string configFilePath{ filePath + configFileName };
    if (!checkFileExists(configFilePath))
    {
        std::cerr << "ICP configuration file '" << configFilePath << "' does not exist." << std::endl;
        return false;
    }

    std::ifstream configFileStream(configFilePath.c_str());
    try
    {
        icp.loadFromYaml(configFileStream);
    }
    catch (const std::exception& ex)
    {
        std::cerr << "ICP could not be configured using file:'" << configFilePath << "'. " << ex.what() << std::endl;
        return false;
    }

    return true;
}

RegistrationTestResult registerPointClouds(const PM::DataPoints& readingPointCloud,
                                           const PM::DataPoints& referencePointCloud,
                                           const PM::AffineTransform& initialGuess_T_origin_read,
                                           PM::ICP& icp)
{
    static constexpr bool reinitializeMatcherWithReference{ true };
    std::stringstream messageStream;

    RegistrationTestResult result;
    try
    {
        // Compute icp.
        const PM::TransformationParameters icp_Tmat_origin_read{ icp.compute(
            readingPointCloud, referencePointCloud, initialGuess_T_origin_read.matrix(), reinitializeMatcherWithReference) };

        // Extract ICP Metrics.
        result.isMaxNumberOfIterationsReached = icp.getMaxNumIterationsReached();
        result.pointCloudOverlap = icp.errorMinimizer->getOverlap();
        result.icpCorrection_T_origin_read.matrix() = initialGuess_T_origin_read.matrix().inverse() * icp_Tmat_origin_read;
        result.icp_T_origin_read.matrix() = icp_Tmat_origin_read;
        // ICP throws an exception if the result is divergence/error.
        // Thus, if we reach this line in the code, it's because the call to compute the transformation with ICP succeeded
        result.success = true;

        // Transform point clouds to allow visualization.
        std::shared_ptr<PM::Transformation> transformator{ PM::get().REG(PM::Transformation).create("RigidTransformation") };
        // We copy and transform the reading point cloud using the initial guess transform from reading to origin frame.
        result.readingPointCloudInOriginFrameFollowingInitialGuess = readingPointCloud;
        transformator->inPlaceCompute(initialGuess_T_origin_read.matrix(), result.readingPointCloudInOriginFrameFollowingInitialGuess);
        // We copy and transform the reading point cloud using the corrected transform from reading to origin frame.
        result.readingPointCloudInOriginFrameFollowingIcpCorrection = readingPointCloud;
        transformator->inPlaceCompute(result.icp_T_origin_read.matrix(), result.readingPointCloudInOriginFrameFollowingIcpCorrection);
    }
    catch (const PM::ConvergenceError& error)
    {
        messageStream << "Experienced a convergence error while running ICP: " << error.what() << std::endl;
        result.success = false;
        std::cerr << messageStream.str();
    }
    catch (const std::runtime_error& error)
    {
        messageStream << "Experienced a runtime error while running ICP: " << error.what() << std::endl;
        result.success = false;
        std::cerr << messageStream.str();
    }

    result.message = messageStream.str();

    return result;
}