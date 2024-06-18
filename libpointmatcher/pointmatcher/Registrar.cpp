#include "PointMatcher.h"

#include <yaml-cpp/yaml.h>

namespace PointMatcherSupport
{
void getNameParamsFromYAML(const YAML::Node& module, std::string& name, Parametrizable::Parameters& params)
{
    if (module.size() != 1)
    {
        // parameter-less entry
        name = module.as<std::string>();
    }
    else
    {
        // get parameters
        YAML::const_iterator mapIt(module.begin());
        name = mapIt->first.as<std::string>();
        for (YAML::const_iterator paramIt = mapIt->second.begin(); paramIt != mapIt->second.end(); ++paramIt)
        {
            auto key = paramIt->first.as<std::string>();
            auto value = paramIt->second.as<std::string>();
            params[key] = value;
        }
    }
}
} // namespace PointMatcherSupport
