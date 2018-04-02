/**
 * This node subscribes to videofeed and kalman predictions and displays them
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

/**
 * Mat object to hold video frame
 */
cv::Mat frame;

/**
 * Coordinates of the buoy
 */
double m_x = 0.0;
double m_y = 0.0;
double m_r = 0.0;

/**
 * VideoFeed Callback function, updates frame, draws last known prediction
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try{
    ROS_INFO("Received a frame\n");
    frame  = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::circle(frame, cv::Point(m_x,m_y), 20, CV_RGB(255,0,0), 2);
    cv::imshow("Window",frame);
    cv::waitKey(1);

  }catch(cv_bridge::Exception e){
    ROS_ERROR("Some error");
  }
  
}

/**
 * Prediction Callback function, updates prediction, draws it on last known frame
 */
void dataCallback(const std_msgs::String::ConstPtr& msg)
{
  /**
   * Extracting prediction values from the string message
   */
	std::string predicted = msg->data.c_str();
	std::string buf; 
	std::stringstream ss(predicted); 
	std::vector<std::string> tokens; 
	while (ss >> buf)
    	tokens.push_back(buf);

    //ROS_INFO("%s",msg->data.c_str());
    m_x = std::strtod(tokens[0].c_str(),NULL);
    m_y = std::strtod(tokens[1].c_str(),NULL);
    m_r = std::strtod(tokens[2].c_str(),NULL);

    /**
     * Update frame
     */
    cv::circle(frame, cv::Point(m_x,m_y), 20, CV_RGB(255,0,0), 2);
    cv::imshow("Window",frame);
    cv::waitKey(1);
    //cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "trackerdisp");
  ros::NodeHandle n;
  cv::namedWindow("Window");
  cv::startWindowThread();

  /**
   * Subscriber for videofeed
   */
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("videofeed", 1000, imageCallback);

  /**
   * Subscriber for tracking predictions
   */
  ros::Subscriber sub2 = n.subscribe("prediction", 1000, dataCallback);

  ros::spin();
  cv::destroyWindow("Window");

  return 0;
}

