
#include "utils_filesystem.h"

#include <iostream>

#include <boost/filesystem.hpp>

bool createDirectory(const std::string& directoryPath)
{
    // If the folder doesn't exist, create it.
    try
    {
        return boost::filesystem::create_directories(directoryPath);
    }
    catch (const boost::filesystem::filesystem_error& exception)
    {
        std::cout << "Caught an exception trying to create directory: " << exception.what();
    }
    std::cout << "Directory '" << directoryPath.c_str() << "' doesn't exist and could not be created";

    return false;
}

bool checkFileExists(const std::string& filePathStr)
{
    const boost::filesystem::path filePath(filePathStr);
    return boost::filesystem::exists(filePath);
}

bool checkDirectoryExists(const std::string& directoryPathStr)
{
    const boost::filesystem::path directoryPath(directoryPathStr);
    return boost::filesystem::is_directory(directoryPath);
}

bool removeDirectory(const std::string& directoryPath)
{
    try
    {
        return boost::filesystem::remove_all(directoryPath);
    }
    catch (const boost::filesystem::filesystem_error& exception)
    {
        std::cout << "Caught an exception trying to remove directory: " << exception.what();
    }
    std::cout << "Directory '" << directoryPath.c_str() << "' doesn't exist and could not be removed";

    return false;
}