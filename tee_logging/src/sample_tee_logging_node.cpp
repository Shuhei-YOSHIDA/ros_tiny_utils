/**
 * @file sample_tee_logging_node.cpp
 */

#include <ros/ros.h>
#include "tee_logging/tee_logging.h"

using namespace tee_logging;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sample_tee_logging_node");
  // Set absolute path, it's better
  TeeLogging tl1("log1.txt");
  TeeLogging tl2("log2.txt");
  TeeLogging tl1_1("log1.txt");

  tl1.log_info("tl1: info-test1");
  tl1.log_info("tl1: info-test2");

  tl2.log_info("tl2: info-test1");
  
  tl1_1.log_info("tl1_1: info-test !");
  return 0;
}
