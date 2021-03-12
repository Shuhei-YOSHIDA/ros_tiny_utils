/**
 * @file sample_resolve_path_node.cpp
 */

#include "tiny_conversion/resolve_path.h"
#include <ros/console.h>
#include <iostream>
#include <vector>

using namespace std;
using namespace tiny_conversion;

int main(int argc, char** argv)
{
  vector<string> sample_path_data = {
    "package://tiny_conversion/src/sample_resolve_path_node.cpp",
    "file://tiny_conversion/package.xml",
    "package://tiny_conversion/",
    "file://tiny_conversion/",
    "",
    "foobar",
    "./package.xml"
  };

  for (auto path : sample_path_data)
  {
    string url = path;
    string resolved_path = resolvePathUrlStr(url);
    ROS_INFO_STREAM("[" << url << "] : [" << resolved_path << "]");
  }

  return 0;
}
