#pragma once

#include <string>

//! Fetches the name of the test case currently running.
std::string fetchTestName();

//! Sets up Randon Number Generation (RNG) with a known seed.
void setUpKnownRNGSeedForTest(const unsigned int seed);

//! Sets up Randon Number Generation (RNG) with a random (time-based) seed.
void setUpRandomRNGSeedForTest();