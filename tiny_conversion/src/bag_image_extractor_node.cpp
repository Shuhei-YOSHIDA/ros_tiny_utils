/**
 * @file bag_image_extractor_node.cpp
 * @brief Extract sensor_msgs/Image topic and save it as image files
 * @note Reference: ros-perception/image_pipeline repository, image_view package
 */

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sys/stat.h>

using namespace std;

void usage()
{
  cout << "usage:" << endl
       << "rosrun tiny_conversion bag_image_extractor_node [bagfile_path] [image_topic_name] [output_dir]" << endl;
  return;
}

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    ROS_FATAL("less arguments");
    usage();
    return -1;
  }

  string bagfile_path(argv[1]);
  string image_topic_name(argv[2]);
  string output_dir(argv[3]);

  ROS_INFO("bagfile_path: %s", bagfile_path.c_str());
  ROS_INFO("image_topic_name: %s", image_topic_name.c_str());
  ROS_INFO("output_dir: %s", output_dir.c_str());

  struct stat statBuf;
  if (stat(output_dir.c_str(), &statBuf) != 0)
  {
    ROS_FATAL("output_dir does not exist");
    usage();
    return -1;
  }

  rosbag::Bag bag;
  try
  {
    bag.open(bagfile_path);
  }
  catch(rosbag::BagException& e)
  {
    ROS_FATAL("Could not open the bagfile");
    cerr << e.what() << endl;
    usage();
    return -1;
  }

  vector<string> topic_names = {image_topic_name};
  rosbag::View view(bag, rosbag::TopicQuery(topic_names));
  for (auto m : view)
  {
    sensor_msgs::Image::ConstPtr img_ptr = m.instantiate<sensor_msgs::Image>();
    if (img_ptr != NULL) continue;

    //ros::Time bag_stamp = m.getTime();
    ros::Time header_stamp = img_ptr->header.stamp;;

    cv::Mat image;
    try
    {
      image = cv_bridge::toCvShare(img_ptr)->image;
    }
    catch(cv_bridge::Exception &e)
    {
      cerr << e.what() << endl;
      ROS_FATAL("Failed to convert img message to cv::Mat");
      return -1;
    }

    if (!image.empty())
    {

      stringstream stamp_ss;
      stamp_ss << header_stamp.sec << "." << std::setw(9)
               << std::setfill('0') << header_stamp.nsec << ".png";
      string filename = output_dir + "/" + stamp_ss.str();
      cv::imwrite(filename, image);
    }
    else
    {
      ROS_ERROR("image is empty");
    }
  }

  return 0;
}
