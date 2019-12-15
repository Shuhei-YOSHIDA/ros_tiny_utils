/**
 * @file sample_tee_logging_node.cpp
 */

#include <ros/ros.h>
#include <sstream>
#include "tee_logging/tee_logging.h"

using namespace std;
using namespace tee_logging;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sample_tee_logging_node");
  // Set absolute path, it's better
  TeeLogging tl1("log1.log");
  TeeLogging tl2("log2.log");
  TeeLogging tl1_1("log1.log");

  tl1.log_info("tl1: info-test1");
  tl1.log_info("tl1: info-test2");

  tl2.log_info("tl2: info-test1");
  
  tl1_1.log_info("tl1_1: info-test !");

  stringstream ss;
  ss << "tl1 stringstream 1";
  tl1.log_info(ss); // string in ss is replaced by "";
  ss << "tl1 stringstream 2";
  tl1.log_info(ss);

  return 0;
}
