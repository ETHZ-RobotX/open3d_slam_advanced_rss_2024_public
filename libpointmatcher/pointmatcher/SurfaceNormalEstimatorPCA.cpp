// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2022,
Yoshua Nava, ANYbotics AG, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "PointMatcher.h"

template<typename T>
typename PointMatcher<T>::Vector PointMatcher<T>::SurfaceNormalEstimatorPCA::extrudeNormalFromIncreasinglyOrderedEigenvectors(
    const typename PointMatcher<T>::Vector& /*eigenValues*/, const typename PointMatcher<T>::Matrix& eigenVectors) noexcept
{
    // Knowing the order of eigenvector/eigenvalue pairs can be beneficial for performance. Eigensolvers like libeigen's SelfAdjointEigenSolver
    // outputs eigenvectors (and eigenvalues pairs based on increasing order of eigenvalue magnitude.
    // Ref: https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html#a62817de3e0cbf009a02c7ece6a0e3d64
    return eigenVectors.col(0);
}

template<typename T>
typename PointMatcher<T>::Vector PointMatcher<T>::SurfaceNormalEstimatorPCA::extrudeNormalFromUnorderedEigenvectors(
    const typename PointMatcher<T>::Vector& eigenValues, const typename PointMatcher<T>::Matrix& eigenVectors) noexcept
{
    // Keep the smallest eigenvector as surface normal
    const Eigen::Index nbEigenCol{ eigenVectors.cols() };
    Eigen::Index smallestId(0);
    T smallestValue(std::numeric_limits<T>::max());
    for (Eigen::Index i = 0; i < nbEigenCol; ++i)
    {
        if (eigenValues(i) < smallestValue)
        {
            smallestId = i;
            smallestValue = eigenValues(i);
        }
    }

    return eigenVectors.col(smallestId);
}

template<typename T>
void PointMatcher<T>::SurfaceNormalEstimatorPCA::orientNormalVectorTowardsSensorFrame(const Vector& sensorPosition,
                                                                                      const Vector& point,
                                                                                      Vector& normalVector) noexcept
{
    // Explanation behind the method.
    //  Observation direction vector = sensorPosition - point
    //  Scale * Cos(Angle between Observation and Normal Vectors) = (Observation vector).dot(Normal vector)
    //  If Cos < 0, Angle between Observation and Normal vectors is > pi/2 radians. Thus, the normal is looking contrary to the sensor position/origin.
    //      Action: Re-orient normal by flipping its sign
    const T scaledCosineOfAngleToSensorFrame{ (sensorPosition - point).dot(normalVector) };
    if (scaledCosineOfAngleToSensorFrame < 0)
    {
        normalVector = -normalVector;
    }
}


template<typename T>
void PointMatcher<T>::SurfaceNormalEstimatorPCA::computeLocalSurfaceDescriptors(const typename PointMatcher<T>::Vector& eigenValues,
                                                                                T& linearity, T& planarity, T& curvature) noexcept
{
    // We take the square root of the eigenvalues, to get the span of the surface.
    const T sigma1{ std::sqrt(eigenValues(0)) };
    const T sigma2{ std::sqrt(eigenValues(1)) };
    const T sigma3{ std::sqrt(eigenValues(2)) };

    // We compute the surface properties based on the span of the surface in different directions.
    linearity = (sigma3 - sigma2) / sigma3;
    planarity = (sigma2 - sigma1) / sigma3;
    curvature = sigma1 / (sigma1 + sigma2 + sigma3);
}


template<typename T>
bool PointMatcher<T>::SurfaceNormalEstimatorPCA::computeNormalVector(const typename PointMatcher<T>::Matrix& points, Vector& normal,
                                                                     T& linearity, T& planarity, T& curvature) noexcept
{
    constexpr T machineEpsilon{ std::numeric_limits<T>::epsilon() };

    const Eigen::Index numberOfPointsToComputeNormal{ points.cols() };
    if (numberOfPointsToComputeNormal < minimumNumberOfPointsToComputeNormal)
    {
        return false;
    }

    const Vector mean{ points.rowwise().mean() };
    const PointMatcher<T>::Matrix NN{ points.colwise() - mean };
    // TODO(ynava) Find out whether we should normalize over N or N-1?
    const FixedSizeMatrix3 covariance{ NN * NN.transpose() / static_cast<T>(numberOfPointsToComputeNormal) };

    // Compute eigenvalues and eigenvectors of the covariance matrix.
    EigenvalueSolver eigenSolver;
    eigenSolver.computeDirect(covariance);
    const PointMatcher<T>::Vector eigenValues{ eigenSolver.eigenvalues().real() };
    const PointMatcher<T>::Matrix eigenVectors{ eigenSolver.eigenvectors().real() };

    // Compute a robust threshold for eigenvalue-based rank computation, inspired on the documentation of
    // the libeigen Eigenvalue solver we use, and on the implementation of the same concept for libeigen SVD solvers.
    // Ref: https://eigen.tuxfamily.org/dox/classEigen_1_1SVDBase.html#a98b2ee98690358951807353812a05c69
    const T premultipliedThreshold{ (static_cast<T>(kPointDimension) * machineEpsilon) * eigenValues(2) };
    const Eigen::Index rankOfC{ (eigenValues.array().abs() > premultipliedThreshold).count() };

    // Validate the rank of the covariance matrix. A rank-defficient matrix would give rise to degenerate eigenvalues/eigenvectors, and wrong normals.
    if (rankOfC + 1 < kPointDimension)
    {
        return false;
    }

    // Compute normal and clamp it to [-1,1] to handle approximation errors
    normal = extrudeNormalFromIncreasinglyOrderedEigenvectors(eigenValues, eigenVectors);

    // Clamp the values of the normal vector to be in the range [-1, 1].
    // TODO(ynava) Wouldn't it be better to perform normalization here?
    normal = normal.cwiseMax(-1.0).cwiseMin(1.0);

    // Compute surface properties of the patch used to extrude the normal.
    computeLocalSurfaceDescriptors(eigenValues, linearity, planarity, curvature);

    return true;
}

template struct PointMatcher<float>::SurfaceNormalEstimatorPCA;
template struct PointMatcher<double>::SurfaceNormalEstimatorPCA;
