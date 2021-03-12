/**
 * @file resolve_path.cpp
 */

#include "tiny_conversion/resolve_path.h"
#include <ros/package.h>
#include <ros/console.h>
#include <boost/filesystem.hpp>
//#include <filesystem>

using namespace std;

vector<string> split(string str, char delimiter)
{
  int first = 0;
  int last = str.find_first_of(delimiter);

  vector<string> result;

  while (first < str.size())
  {
    string subStr(str, first, last - first);
    result.push_back(subStr);

    first = last + 1;
    last = str.find_first_of(delimiter, first);

    if (last == string::npos)
    {
      last = str.size();
    }
  }

  //if (str.find_last_of(delimiter) == str.size()-1)
  //{
    //result.push_back("");
  //}

  return result;
}


namespace tiny_conversion

{

string resolvePathUrlStr(const string& url)
{
  vector<string> splited = split(url, '/');

  boost::filesystem::path path;
  if (url.find("package://") == 0) // Case-insensitive?
  {
    string package_name = splited[2];
    string file_name;
    for (int i = 3; i < splited.size(); i++)
    {
      if (i == 3) file_name += splited[i];
      else file_name += ("/" + splited[i]);
    }
    path = ros::package::getPath(package_name);
    path = path / file_name;
  }
  else if (url.find("file://") == 0)
  {
    for (int i = 2; i < splited.size(); i++)
    {
      if (i == 2) path += splited[i];
      else path += ("/" + splited[i]);
    }
  }
  else
  {
    path = boost::filesystem::path(url);
    if (!boost::filesystem::exists(path))
    {
      ROS_ERROR("Invalid or unsupported URL: '%s'", url.c_str());
      return "";
    }
  }

  string resolved_pathstr = path.string();

  return resolved_pathstr;
}

//filesystem::path resolvePath(const string& path)
//{
  //filesystem::path resolved_path;

  //return resolved_path;
//}

}
