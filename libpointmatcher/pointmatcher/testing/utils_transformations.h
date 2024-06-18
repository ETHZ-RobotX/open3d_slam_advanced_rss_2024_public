#pragma once

#include <cmath>

#include "../PointMatcher.h"

#include "TransformationError.h"

/**
 * @brief Computes the error between two transformations A and B
 * @remark It bases the error computation on the definition of delta = inverse(T_A) * T_b
 * 
 * @param transformA 
 * @param transformB 
 * @return TransformationError 
 */
TransformationError computeError(const PM::AffineTransform& transformA, const PM::AffineTransform& transformB);

/**
 * @brief Checks whether two transforms A and B are approximately equal, up to an error tolerance.
 * 
 * @param transformA    Transform A.
 * @param transformB    Transform B.
 * @param epsilon       Error tolerance.
 * @param message       Error message, if any.
 * @return bool         Whether the transformations are approximately equal.
 */
bool isApprox(const PM::AffineTransform& transformA, const PM::AffineTransform& transformB, const PM::ScalarType epsilon,
              std::string& message);