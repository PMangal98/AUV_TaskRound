/**
 * This node publishes the video feed
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  /**
   * Publisher for video feed
   */
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("videofeed", 1000);
    
  /**
   * Mat object to hold video frame
   */
  cv:: Mat frame;
  cv:: VideoCapture cap("/home/param/Desktop/OpenCV1/Task Buoy/frontcam.avi");
  
  /**
   * The video is at 25 Frames per second
   */
  ros::Rate loop_rate(40);

  int count = 0;
  while (n.ok())
  {
    try{
      cap>>frame;
    }catch(std::exception e){
      ROS_INFO("Some error");
      return 0;
    }

    if(frame.empty()){
      ROS_INFO("End of video");
      break;
    }

    /**
     * Conversion to ROS message
     */
    sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
  
    //cv::imshow("Publisher window",frame);
    
    /*
     * Publishing video at 25 frames per second.
    **/
    cv::waitKey(40);

    ROS_INFO("Published a frame %d\n",count);

    /**
     * Publish message
     */
    pub.publish(msg_image);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
