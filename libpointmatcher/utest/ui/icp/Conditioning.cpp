
#include <vector>

#include <pointmatcher/testing/utils_filesystem.h>
#include <pointmatcher/testing/utils_geometry.h>
#include <pointmatcher/testing/utils_gtest.h>
#include <pointmatcher/testing/RegistrationTestCase.h>
#include <pointmatcher/testing/RegistrationTestResult.h>
#include <pointmatcher/testing/utils_registration.h>
#include <pointmatcher/testing/utils_transformations.h>

#include "../../utest.h"

class ICPConditioningTest : public ::testing::Test
{
public:
    ICPConditioningTest() = default;

    //! Sets up the test, including random number generation.
    void SetUp() override { setUpRandomRNGSeedForTest(); }

    /**
     * @brief Sets up different test cases for validating the conditioning of ICP registration.
     * 
     * @param nbPoints  Number of points in the point clouds to register.
     * @param scale   Scale factor used to amplify the distances and sizes of the shapes to register.   
     * @param translationNoiseStdDev  Standard deviation intrinsic to the translation component of the registration initial guess.
     * @param rotationNoiseStdDevInDeg  Standard deviation intrinsic to the rotation component of the registration initial guess.
     * @param useSamePointCloudsForReferenceAndReading  Whether the same point clouds should be used as reading and reference.
     */
    void setUpTestCases(const PM::DataPoints::Index nbPoints,
                        const PM::ScalarType scale,
                        const PM::ScalarType translationNoiseStdDev,
                        const PM::ScalarType rotationNoiseStdDevInDeg,
                        const bool useSamePointCloudsForReferenceAndReading)
    {
        // We build random translation and rotations for the error inherent to the initial guess.
        const PM::StaticCoordVector initialGuess_t_error{ buildRandomVectorFromStdDev(translationNoiseStdDev) };
        const PM::Quaternion initialGuess_R_error{ buildRandomQuaternionFromStdDev(convertDegreesToRadians(rotationNoiseStdDevInDeg)) };

        /* In the lines below we use the terms Shift and Displacement.
           But what is the difference between them?
              *Displacement* refers to a relative pose difference between two objects.
              Now, In the papers by Szymon Rusinkiewicz et al (e.g. Geometrically Stable Sampling for the ICP Algorithm) 
              they refer to *Shift* as the static displacement that both the reading and the reference point cloud bear 
              from a third frame, which we call the Origin.

              Thus, in both cases we are referring to displacements, but Shift is a special term introduced to make explicit that there is a
            shared property between both objects that we aim to register.
        */

        testCases.emplace_back("BoxClouds_NoReferenceShift_NoDisplacement",
                               nbPoints,
                               scale,
                               /* Origin->Reference shift */
                               PM::StaticCoordVector::Zero(),
                               PM::Quaternion::Identity(),
                               /* Reference->reading transformation */
                               PM::StaticCoordVector::Zero(),
                               PM::Quaternion::Identity(),
                               /* Initial guess error */
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_1mXReferenceShift_NoDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector(1.0, 0, 0),
                               PM::Quaternion::Identity(),
                               PM::StaticCoordVector::Zero(),
                               PM::Quaternion::Identity(),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_1000mXReferenceShift_NoDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector(1000.0, 0, 0),
                               PM::Quaternion::Identity(),
                               PM::StaticCoordVector::Zero(),
                               PM::Quaternion::Identity(),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_10DegYawReferenceShift_NoDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector::Zero(),
                               buildQuaternionFromRPY(0.0, 0.0, 10),
                               PM::StaticCoordVector::Zero(),
                               PM::Quaternion::Identity(),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_30DegYawReferenceShift_NoDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector::Zero(),
                               buildQuaternionFromRPY(0.0, 0.0, 30),
                               PM::StaticCoordVector::Zero(),
                               PM::Quaternion::Identity(),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_1mX10DegYawReferenceShift_NoDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector(1.0, 0.0, 0.0),
                               buildQuaternionFromRPY(0.0, 0.0, 10),
                               PM::StaticCoordVector::Zero(),
                               PM::Quaternion::Identity(),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_1mX30DegYawReferenceShift_NoDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector(1.0, 0.0, 0.0),
                               buildQuaternionFromRPY(0.0, 0.0, 30),
                               PM::StaticCoordVector::Zero(),
                               PM::Quaternion::Identity(),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_NoReferenceShift_10DegYawDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector::Zero(),
                               PM::Quaternion::Identity(),
                               PM::StaticCoordVector::Zero(),
                               buildQuaternionFromRPY(0.0, 0.0, 10),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_NoYawReferenceShift_30DegYawDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector::Zero(),
                               PM::Quaternion::Identity(),
                               PM::StaticCoordVector::Zero(),
                               buildQuaternionFromRPY(0.0, 0.0, 30),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_1mXReferenceShift_10DegYawDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector(1.0, 0, 0),
                               PM::Quaternion::Identity(),
                               PM::StaticCoordVector::Zero(),
                               buildQuaternionFromRPY(0.0, 0.0, 10),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_100mX-3000mZReferenceShift_10DegYawDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector(100.0, 0, -3000),
                               PM::Quaternion::Identity(),
                               PM::StaticCoordVector::Zero(),
                               buildQuaternionFromRPY(0.0, 0.0, 10),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_100mX+3000mY+30DegYawReferenceShift_10DegYawDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector(100.0, 3000, 0),
                               PM::Quaternion::Identity(),
                               PM::StaticCoordVector::Zero(),
                               buildQuaternionFromRPY(0.0, 0.0, 30),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_10000mX+3000mY-3500Z+30DegPitchRollYawYawReferenceShift_10DegYawDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector(10000.0, 3000, -3500),
                               PM::Quaternion::Identity(),
                               PM::StaticCoordVector::Zero(),
                               buildQuaternionFromRPY(30, 30, 30),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_30DegYawReferenceShift_10DegYawDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector::Zero(),
                               buildQuaternionFromRPY(0.0, 0.0, 30),
                               PM::StaticCoordVector::Zero(),
                               buildQuaternionFromRPY(0.0, 0.0, 10),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_1mX30DegYawReferenceShift_10DegYawDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector(1.0, 0.0, 0.0),
                               buildQuaternionFromRPY(0.0, 0.0, 30),
                               PM::StaticCoordVector::Zero(),
                               buildQuaternionFromRPY(0.0, 0.0, 10),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_2mX30DegYawReferenceShift_10DegYawDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector(2.0, 0.0, 0.0),
                               buildQuaternionFromRPY(0.0, 0.0, 30),
                               PM::StaticCoordVector::Zero(),
                               buildQuaternionFromRPY(0.0, 0.0, 10),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_2mX30DegYawReferenceShift_-1mY10DegYawDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector(2.0, 0.0, 0.0),
                               buildQuaternionFromRPY(0.0, 0.0, 30),
                               PM::StaticCoordVector(0.0, -1.0, 0.0),
                               buildQuaternionFromRPY(0.0, 0.0, 10),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_10mX30DegYawReferenceShift_-1mY10DegYawDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector(10.0, 0.0, 0.0),
                               buildQuaternionFromRPY(0.0, 0.0, 30),
                               PM::StaticCoordVector(0.0, -1.0, 0.0),
                               buildQuaternionFromRPY(0.0, 0.0, 10),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_10mY-30DegYawReferenceShift_-1mY10DegYawDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector(0.0, 10.0, 0.0),
                               buildQuaternionFromRPY(0.0, 0.0, -30),
                               PM::StaticCoordVector(0.0, -1.0, 0.0),
                               buildQuaternionFromRPY(0.0, 0.0, 10),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
        testCases.emplace_back("BoxClouds_10mY-30DegYawReferenceShift_-1mY-50DegYawDisplacement",
                               nbPoints,
                               scale,
                               PM::StaticCoordVector(0.0, 10.0, 0.0),
                               buildQuaternionFromRPY(0.0, 0.0, -30),
                               PM::StaticCoordVector(0.0, -1.0, 0.0),
                               buildQuaternionFromRPY(0.0, 0.0, -50),
                               initialGuess_t_error,
                               initialGuess_R_error,
                               useSamePointCloudsForReferenceAndReading);
    }

    bool dumpTestToFilesystem(const std::string& testSubfolder,
                              const RegistrationTestCase& testCase,
                              const RegistrationTestResult& testResult)
    {
        const std::string targetDirectory(testDataFolderPath + testSubfolder + "/" + testCase.name + "/");

        // Remove directory, just in case it already exists.
        if (checkDirectoryExists(targetDirectory))
        {
            removeDirectory(targetDirectory);
        }

        // Create directory.
        if (!createDirectory(targetDirectory))
        {
            return false;
        }

        // Save the shape point clouds to disk.
        testCase.referencePointCloudInOriginFrame.save(targetDirectory + "referencePointCloudInOriginFrame.ply");
        testCase.readingPointCloudInReadingFrame.save(targetDirectory + "readingPointCloudInReadingFrame.ply");
        testResult.readingPointCloudInOriginFrameFollowingInitialGuess.save(targetDirectory
                                                                            + "readingPointCloudInOriginFrameFollowingInitialGuess.ply");
        testResult.readingPointCloudInOriginFrameFollowingIcpCorrection.save(targetDirectory
                                                                             + "readingPointCloudInOriginFrameFollowingIcpCorrection.ply");

        return true;
    }

    void runTestCases(const std::string& testName, const PM::DataPoints::Index nbPoints, const PM::ScalarType scale,
                      const PM::ScalarType translationNoiseStdDev, const PM::ScalarType rotationNoiseStdDevInDeg,
                      const bool useSamePointCloudsForReferenceAndReading, const PM::ScalarType epsilonError)
    {
        ASSERT_TRUE(configureIcp(dataPath, icpConfigFileName, icp));

        setUpTestCases(nbPoints, scale, translationNoiseStdDev, rotationNoiseStdDevInDeg, useSamePointCloudsForReferenceAndReading);

        TransformationError avgError;
        for (const auto& testCase : testCases)
        {
            std::string outputMessage;
            const RegistrationTestResult testResult{ registerPointClouds(testCase.readingPointCloudInReadingFrame,
                                                                         testCase.referencePointCloudInOriginFrame,
                                                                         testCase.initialGuess_T_origin_read,
                                                                         icp) };

            const bool areResultsApproxEqual(isApprox(testCase.T_origin_read, testResult.icp_T_origin_read, epsilonError, outputMessage));
            EXPECT_TRUE(areResultsApproxEqual) << "\n"
                                               << "\033[1;33m"
                                               << "Test case"
                                               << "\033[0m \n"
                                               << testCase << "\nTest results:\n"
                                               << testResult << "\nDeltas:\n"
                                               << outputMessage << "\n\n\n";
            const TransformationError error{ computeError(testCase.T_origin_read, testResult.icp_T_origin_read) };
            avgError += error;

            if (saveTestDataToDisk && !areResultsApproxEqual)
            {
                dumpTestToFilesystem(testName, testCase, testResult);
            }
        }

        /**
         * Although this unit test checks that the error of each test case is lower than a threshold, and that immediately puts a bound on the peak error over the test case,
         * it can be useful for developers to look at what's the average error over all the tests.
         * Thus, you can enable printing the average error by uncommenting the lines below.
         */
        avgError /= testCases.size();
        // std::cout << "### Average error ###\n" << avgError;
    }


//! Whether test data should be saved to disk.
#ifdef SAVE_TEST_DATA_TO_DISK
    const bool saveTestDataToDisk{ true };
#else
    const bool saveTestDataToDisk{ false };
#endif
    const std::string testDataFolderPath{ "/tmp/libpointmatcher/unit_tests/icp_conditioning/" };

    //! Test cases.
    std::vector<RegistrationTestCase> testCases;

    //! ICP registration pipeline.
    PM::ICP icp;
    //! Configuration file, expected to be stored under 'examples/data/'
    std::string icpConfigFileName{ "icp_point_to_plane.yaml" };
};

// This test validates that ICP is effective at computing a transformation for two identical box-shaped point clouds with no noise in the initial guess.
TEST_F(ICPConditioningTest, RegistrationSameBoxPointCloudsNoNoiseIG)
{
    const std::string testName{ fetchTestName() };

    // Number of points in each point cloud.
    const PM::DataPoints::Index nbPoints{ 10000 };
    const PM::ScalarType scale{ 1.0 };
    const bool useSamePointCloudsForReferenceAndReading{ true };
    const PM::ScalarType epsilonError{ 1e-6 };

    // Initial guess noise, in terms of its standard deviations.
    const PM::ScalarType translationNoiseStdDev{ 0 };
    const PM::ScalarType rotationNoiseStdDevInDeg{ 0 };

    // Run tests.
    runTestCases(testName,
                 nbPoints,
                 scale,
                 translationNoiseStdDev,
                 rotationNoiseStdDevInDeg,
                 useSamePointCloudsForReferenceAndReading,
                 epsilonError);
}

// This test validates that ICP is effective at computing a transformation for two identical box-shaped point clouds with a 50x scale increase.
// We expect the numerical accuracy of ICP to decrease roughlu proportional to the orders of magnitude of the scale increase.
TEST_F(ICPConditioningTest, RegistrationSameBoxPointCloudsNoNoiseIGScale50)
{
    const std::string testName{ fetchTestName() };

    // Number of points in each point cloud.
    const PM::DataPoints::Index nbPoints{ 10000 };
    const PM::ScalarType scale{ 50.0 };
    const bool useSamePointCloudsForReferenceAndReading{ true };
    const PM::ScalarType epsilonError{ 1e-4 };

    // Initial guess noise, in terms of its standard deviations.
    const PM::ScalarType translationNoiseStdDev{ 0 };
    const PM::ScalarType rotationNoiseStdDevInDeg{ 0 };

    // Run tests.
    runTestCases(testName,
                 nbPoints,
                 scale,
                 translationNoiseStdDev,
                 rotationNoiseStdDevInDeg,
                 useSamePointCloudsForReferenceAndReading,
                 epsilonError);
}

// This test validates that ICP is effective at computing a transformation for two different box-shaped point clouds with no noise in the initial guess.
TEST_F(ICPConditioningTest, RegistrationDifferentBoxPointCloudsNoNoiseIG)
{
    const std::string testName{ fetchTestName() };

    // Number of points in each point cloud.
    const PM::DataPoints::Index nbPoints{ 10000 };
    const PM::ScalarType scale{ 1.0 };
    const bool useSamePointCloudsForReferenceAndReading{ false };
    const PM::ScalarType epsilonError{ 1e-5 };

    // Initial guess noise, in terms of its standard deviations.
    const PM::ScalarType translationNoiseStdDev{ 0 };
    const PM::ScalarType rotationNoiseStdDevInDeg{ 0 };

    // Run tests.
    runTestCases(testName,
                 nbPoints,
                 scale,
                 translationNoiseStdDev,
                 rotationNoiseStdDevInDeg,
                 useSamePointCloudsForReferenceAndReading,
                 epsilonError);
}

// This test validates that ICP is effective at computing a transformation for two identical box-shaped point clouds with noise in the initial guess.
TEST_F(ICPConditioningTest, RegistrationSameBoxPointCloudsNoiseIG)
{
    const std::string testName{ fetchTestName() };

    // Number of points in each point cloud.
    const PM::DataPoints::Index nbPoints{ 10000 };
    const PM::ScalarType scale{ 1.0 };
    const bool useSamePointCloudsForReferenceAndReading{ true };
    const PM::ScalarType epsilonError{ 1e-6 };

    // Initial guess noise, in terms of its standard deviations.
    const PM::ScalarType translationNoiseStdDev{ 1.0 };
    const PM::ScalarType rotationNoiseStdDevInDeg{ 30.0 };

    // Run tests.
    runTestCases(testName,
                 nbPoints,
                 scale,
                 translationNoiseStdDev,
                 rotationNoiseStdDevInDeg,
                 useSamePointCloudsForReferenceAndReading,
                 epsilonError);
}

// This test validates that ICP is effective at computing a transformation for two different box-shaped point clouds with noise in the initial guess.
TEST_F(ICPConditioningTest, RegistrationDifferentBoxPointCloudsNoiseIG)
{
    const std::string testName{ fetchTestName() };

    // Number of points in each point cloud.
    const PM::DataPoints::Index nbPoints{ 10000 };
    const PM::ScalarType scale{ 1.0 };
    const bool useSamePointCloudsForReferenceAndReading{ false };
    const PM::ScalarType epsilonError{ 1e-5 };

    // Initial guess noise, in terms of its standard deviations.
    const PM::ScalarType translationNoiseStdDev{ 0.135 };
    const PM::ScalarType rotationNoiseStdDevInDeg{ 20 };

    // Run tests.
    runTestCases(testName,
                 nbPoints,
                 scale,
                 translationNoiseStdDev,
                 rotationNoiseStdDevInDeg,
                 useSamePointCloudsForReferenceAndReading,
                 epsilonError);
}