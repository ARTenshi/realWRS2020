// C++ standard headers
#include <exception>
#include <string>
#include <cmath>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <ros/topic.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string rgbName         	= "RGB Source";
static const std::string imageTopic      	= "/hsrb/head_rgbd_sensor/rgb/image_rect_color";

ros::Time latestImageStamp;

// ROS call back for every new image received
void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
  latestImageStamp = imgMsg->header.stamp;

  cv_bridge::CvImagePtr cvImgPtr;

  cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
  cv::imshow(rgbName, cvImgPtr->image);
  cv::waitKey(15);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "rgbd_src_node");

  ROS_INFO("Starting rgbd_src_node application ...");
 
  ros::NodeHandle nh;

  // Precondition: Valid clock
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
   ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Create the window to show HSR camera images
  cv::namedWindow(rgbName, cv::WINDOW_AUTOSIZE);

  // Define ROS topic from where HSR publishes images
  image_transport::ImageTransport it(nh);
  // use compressed image transport to use less network bandwidth
  image_transport::TransportHints transportHint("compressed");

  ROS_INFO_STREAM("Subscribing to " << imageTopic << " ...");
  image_transport::Subscriber subrgb = it.subscribe(imageTopic, 1, imageCallback, transportHint);

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::spin();

  cv::destroyWindow(rgbName);

  return EXIT_SUCCESS;
}
