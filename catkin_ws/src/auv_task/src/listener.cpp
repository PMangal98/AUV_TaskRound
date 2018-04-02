/**
 * This node subscribes to talker and publishes measurements on buoy coordinates
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <vector>
#include <iostream>
#include <math.h>

/**
 * Publisher to publish measurements
 */

ros::Publisher publ;

/**
 * Callback Function for the subscription
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try{
    //ROS_INFO("Received a frame\n");
    
    /**
     * Acquiring the frame
     */  
    cv::Mat frame  = cv_bridge::toCvShare(msg, "bgr8")->image;
    
    /**
     * Noise Removal
     */
    cv::Mat blur;
    cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
      
    /**
     * Conversion to HSV
     */
    cv::Mat frmHsv;
    cv::cvtColor(blur, frmHsv, CV_BGR2HSV);

    /**
     * Separating out Hue and Saturation Channels
     */
    cv::Mat frameSat;
    std::vector<cv::Mat> channels;
    split(frmHsv,channels);
    channels[0].copyTo(frmHsv);
    channels[1].copyTo(frameSat);

    /**
     * Red Hue Range
     */
    int redRange = 20;

    cv::Scalar hsv_upper_l(180-redRange);
    cv::Scalar hsv_upper_h(180);
    cv::Mat red_hue_upper;
    inRange(frmHsv, hsv_upper_l, hsv_upper_h, red_hue_upper);

    cv::Scalar hsv_lower_l(0);
    cv::Scalar hsv_lower_h(0+redRange);
    cv::Mat red_hue_lower;
    inRange(frmHsv, hsv_lower_l, hsv_lower_h, red_hue_lower);

    /**
     * Saturation Range to help separate out the reflection
     */
    cv::Scalar sat_l(90);
    cv::Scalar sat_h(255);
    cv::Mat actual_sat;
    inRange(frameSat, sat_l, sat_h, actual_sat);

    /**
     * Binarization
     */
    cv::Mat rangeRes = (red_hue_lower | red_hue_upper)&actual_sat;

    /**
     * Improving output
     */
    cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 5);
    cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 5);

    /**
     * Contour Detection
     */
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

    /**
     * Vector to hold buoy contour if it is found
     */
    std::vector<std::vector<cv::Point> > buoys;
    std::vector<cv::Rect> buoyBox; 

    /**
     * Finding contour with max area, that would be our buoy
     */
    double area = -1.0f;
    int index = -1;

    for (size_t i = 0; i < contours.size(); i++)
    {
        if(area<cv::contourArea(contours[i])){
            area = cv::contourArea(contours[i]);
            index = i;
        }
    }

    if(index!=-1)
    {
      /**
       * Buoy was found
       */
      buoys.push_back(contours[index]);
      buoyBox.push_back(cv::boundingRect(contours[index]));
      
      /**
       * Drawing contours, bounding box and coordinates on the frame.
       */
      cv::drawContours(frame, buoys, 0, CV_RGB(20,150,20), 5);
      cv::rectangle(frame, buoyBox[0], CV_RGB(0,255,0), 2);

      cv::Point center;
      center.x = buoyBox[0].x + buoyBox[0].width / 2;
      center.y = buoyBox[0].y + buoyBox[0].height / 2;
      cv::circle(frame, center, 2, CV_RGB(20,150,20), -1);

      std::stringstream sstr;
      sstr << "(" << center.x << "," << center.y << ")";
      cv::putText(frame, sstr.str(),
                  cv::Point(center.x + 3, center.y - 3),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);

      /*
       * These are the measurements
       */
      double x_CoM = center.x;
      double y_CoM = center.y;
      double r_CoM = sqrt(buoyBox[0].height*buoyBox[0].height + buoyBox[0].width*buoyBox[0].width)/2;

      /**
       * Constructing the string to be published
       */
      std_msgs::String dataMessage;
      std::stringstream dataBuoy;
      dataBuoy<<x_CoM<<" "<<y_CoM<<" "<<r_CoM;
      dataMessage.data = dataBuoy.str();

      /**
       * Publishing the message
       */
      publ.publish(dataMessage);
      ROS_INFO("FOUND");
    }
    else
    {
      /**
       * Buoy was not found
       */
      std_msgs::String dataMessage;
      std::stringstream dataBuoy;
      dataBuoy<<"NOT_FOUND";
      dataMessage.data = dataBuoy.str();
      publ.publish(dataMessage);
      ROS_INFO("NOT_FOUND");
    }

    //cv::imshow("Window",frame);
    //cv::waitKey(1);

  }catch(cv_bridge::Exception e){
    ROS_ERROR("Some error");
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  /**
   * Window to display the received video feed
   */
  cv::namedWindow("Window");
  cv::startWindowThread();

  /**
   * Subscriber for video feed
   */
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("videofeed", 1000, imageCallback);
  
  /**
   * Publisher for CoM and R
   * The publisher publishes whenever it receives a video frame, it does not have an independent clock
   */
  publ = n.advertise<std_msgs::String>("data", 1000);
  ros::spin();

  cv::destroyWindow("Window");

  return 0;
}
