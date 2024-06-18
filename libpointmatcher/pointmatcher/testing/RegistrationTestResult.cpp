#include "RegistrationTestResult.h"

#include "utils_geometry.h"

std::ostream& operator<<(std::ostream& ostream, const RegistrationTestResult& testResult)
{
    ostream << "Delta Transformation with respect to the initial guess computed by ICP (icpCorrection_T_origin_read):\n"
            << testResult.icpCorrection_T_origin_read.matrix() << "\n";
    ostream << "ICP-Corrected Transformation from reading to coordinate frame origin (icp_T_origin_read):\n"
            << testResult.icp_T_origin_read.matrix() << "\n";
    ostream << "Success = " << (testResult.success ? "true" : "false") << "\n";
    if (!testResult.success)
    {
        ostream << "    Reason = " << testResult.message << "\n";
    }

    return ostream;
}
