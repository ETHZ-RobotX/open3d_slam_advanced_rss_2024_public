#pragma once

#include <string>

//! Creates a directory. If the directory to be created contains new sub-folders, they will also be created.
bool createDirectory(const std::string& directoryPath);

//! Checks if a file exists, based on its path.
bool checkFileExists(const std::string& filePathStr);

//! Checks if a directory exists, based on its path.
bool checkDirectoryExists(const std::string& directoryPathStr);

//! Remove a directory. The operation is applied recursively.
bool removeDirectory(const std::string& directoryPath);