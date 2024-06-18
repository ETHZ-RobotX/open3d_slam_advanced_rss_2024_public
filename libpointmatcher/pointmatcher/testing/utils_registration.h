#pragma once

#include <string>

#include "../PointMatcher.h"

#include "typedefs.h"
#include "RegistrationTestResult.h"

//! Configures ICP based on a config file.
bool configureIcp(const std::string& filePath, const std::string& configFileName, PM::ICP& icp);

//! Registers two point clouds based on an initial guess transformation.
RegistrationTestResult registerPointClouds(const PM::DataPoints& readingPointCloud,
                                           const PM::DataPoints& referencePointCloud,
                                           const PM::AffineTransform& initialGuess_T_origin_read,
                                           PM::ICP& icp);
