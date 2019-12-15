tee_logging
====

`tee_logging::TeeLogging` class supplies methods to output log-message to standard output and a file at the same time.

## How to use

```cpp
TeeLogging tl1(getenv("HOME")+"/.ros/log1.log");
TeeLogging tl2(getenv("HOME")+"/.ros/log2.log");
TeeLogging tl1_(getenv("HOME")+"/.ros/log1.log");

// execute ROS_INFO("log1: info-message") and write it to log1.log
tl1.log_info("log1: info-message"); 
// execute ROS_ERROR("log1: info-message") and write it to log1.log
tl1.log_error("log1: error-message");

// execute ROS_INFO("log2: info-message") and write it to log2.log
tl2.log_info("log2: info-message");

// only execute ROS_INFO("info-message")
tl1_.log_info("info-message");

std::stringstream ss;
ss << "stream " << "message";

// output "stream message"
std::cout << ss.str() << std::endl;
// execute ROS_INFO("stream message") and write it to log1.log
tl1.log_info(ss);
// content of stringstream is replaced by "".
std::cout << ss.str() << std::endl; // THIS OUTPUTS just ""(no-content)
```
