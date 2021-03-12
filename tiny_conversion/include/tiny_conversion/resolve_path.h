/**
 * @file resolve_path.h
 */

#ifndef INCLUDE_TINY_CONVERSION_RESOLVE_PATH_H
#define INCLUDE_TINY_CONVERSION_RESOLVE_PATH_H

#include <string>
//#include <boost/filesystem.hpp>
//#include <filesystem>

namespace tiny_conversion
{

std::string resolvePathUrlStr(const std::string& url);
//std::filesystem::path resolvePath(const std::string& path);
//boost::filesystem::path resolvePath(const std::string& path);

}

#endif /* INCLUDE_TINY_CONVERSION_RESOLVE_PATH_H */
