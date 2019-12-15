/**
 * @file tee_logging.cpp
 */

#include "tee_logging/tee_logging.h"

using namespace std;
using namespace tee_logging;

TeeLogging::TeeLogging(string filepath) : filepath_(filepath)
{
  auto itr = file_set_.find(filepath);
  if (itr == file_set_.end())
  {
    file_set_.insert(filepath);
    ofs_.open(filepath);
  }
  else
  {
    ROS_WARN("file path -%s- is already opened by other instance of TeeLogging", filepath.c_str());
    filepath_ = "";
  }
}

TeeLogging::~TeeLogging()
{
  ofs_.close();
  if (filepath_ != "")
  {
    file_set_.erase(filepath_);
  }
}

void TeeLogging::log_debug(const std::string& message)
{
  ROS_DEBUG("%s", message.c_str());
  ofs_ << message << endl;
}

void TeeLogging::log_info(const std::string& message)
{
  ROS_INFO("%s", message.c_str());
  ofs_ << message << endl;
}

void TeeLogging::log_warn(const std::string& message)
{
  ROS_WARN("%s", message.c_str());
  ofs_ << message << endl;
}

void TeeLogging::log_error(const std::string& message)
{
  ROS_ERROR("%s", message.c_str());
  ofs_ << message << endl;
}

void TeeLogging::log_fatal(const std::string& message)
{
  ROS_FATAL("%s", message.c_str());
  ofs_ << message << endl;
}

