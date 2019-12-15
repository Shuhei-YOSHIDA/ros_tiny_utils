/**
 * @file tee_logging.h
 */

#ifndef INCLUDE_TEE_LOGGING_TEE_LOGGING_H
#define INCLUDE_TEE_LOGGING_TEE_LOGGING_H

#include <ros/console.h>
#include <fstream>
#include <string>
#include <set>

namespace tee_logging
{

/**
 * @note * ios::app でファイルを開く
 *       * 同一ファイルに同時に書き込む問題の対策が必要
 *       * ファイル区別のために絶対パスを
 */
class TeeLogging
{
private:
  std::ofstream ofs_;
  // Not to access same file path
  static std::set<std::string> file_set_;
  std::string filepath_;

public:
  TeeLogging(std::string filepath = "");
  ~TeeLogging();

  void log_debug(const std::string& message);
  void log_info(const std::string& message);
  void log_warn(const std::string& message);
  void log_error(const std::string& message);
  void log_fatal(const std::string& message);

  /// @brief stringstream is filled by "" after these.
  void log_debug(std::stringstream& ss);
  void log_info(std::stringstream& ss);
  void log_warn(std::stringstream& ss);
  void log_error(std::stringstream& ss);
  void log_fatal(std::stringstream& ss);

//  void log_debug_once(const std::string& message);
//  void log_info_once(const std::string& message);
//  void log_warn_once(const std::string& message);
//  void log_error_once(const std::string& message);
//  void log_fatal_once(const std::string& message);
};

std::set<std::string> TeeLogging::file_set_ = std::set<std::string>();

} // namespace tee_logging
#endif /* INCLUDE_TEE_LOGGING_TEE_LOGGING_H */
