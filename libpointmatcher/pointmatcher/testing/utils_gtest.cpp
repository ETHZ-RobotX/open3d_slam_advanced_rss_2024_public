
#include "utils_gtest.h"

#include <chrono>
#include <random>

#include <gtest/gtest.h>

std::string fetchTestName()
{
    // Reference: https://github.com/google/googletest/blob/main/docs/advanced.md#getting-the-current-tests-name
    const testing::TestInfo* const test_info{ testing::UnitTest::GetInstance()->current_test_info() };
    if (test_info == nullptr)
    {
        return "";
    }

    return test_info->name();
}

// Sets up Randon Number Generation (RNG) with a known seed.
void setUpKnownRNGSeedForTest(const unsigned int seed)
{
    // Seed random number generator and record value of seed.
    srand(seed);
    ::testing::Test::RecordProperty("random_seed", seed);
}

// Sets up Randon Number Generation (RNG) with a random (time-based) seed.
void setUpRandomRNGSeedForTest()
{
    // Seed random number generator and record value of seed.
    const unsigned int seed{ static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()) };
    srand(seed);
    ::testing::Test::RecordProperty("random_seed", seed);
}