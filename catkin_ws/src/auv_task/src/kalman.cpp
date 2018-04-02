/**
 * This node subscribes to listener and talker and published Kalman predictoins
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
#include <string>
#include <stdlib.h>

int stateSize = 6;  //size of state
int measSize = 3;   //size of measurement
int contrSize = 0;  //size of control

unsigned int type = CV_32F;

cv::KalmanFilter kf;
cv::Mat state(stateSize, 1, type);      // [x,y,r,v_x,v_y,v_r]
cv::Mat meas(measSize, 1, type);        // [z_x,z_y,z_r]
cv::Mat procNoise(stateSize, 1, type);  // [E_x,E_y,E_r,E_v_x,E_v_y,E_v_r]

/**
 * Boolean pertaining to whether the buoy was found in an earlier frame
 */
bool found = false;

/**
 * No of frames since buoy was last detected
 */
int notFoundCount = 0;

/**
 * Callback Function for the subscription
 */
void dataCallback(const std_msgs::String::ConstPtr& msg){
	/**
     * Finding Contour with max area, if there are ant contours
     */
	//ROS_INFO("=================CALLBACK===============================");
        
	std::string measured = msg->data.c_str();
	if(!(measured=="NOT_FOUND")){ //
		/**
         * The buoy was found in the given frame
         */

		/**
         * Extracting measurements from string
         */
		std::string buf; 
    	std::stringstream ss(measured); 
    	std::vector<std::string> tokens; 
    	while (ss >> buf)
        	tokens.push_back(buf);

        //ROS_INFO("%s",msg->data.c_str());
        double m_x = std::strtod(tokens[0].c_str(),NULL);
        double m_y = std::strtod(tokens[1].c_str(),NULL);
        double m_r = std::strtod(tokens[2].c_str(),NULL);

        ROS_INFO("%lf %lf %lf",m_x,m_y,m_r);


        notFoundCount = 0;

        /**
 		 * Getting the measurements
 		 */
        meas.at<float>(0) = m_x;
        meas.at<float>(1) = m_y;
        meas.at<float>(2) = m_r;

        if(!found){
	        /**
 			 * The buoy was either lost or was never located so we have no values of state in place
 			 */
	        kf.errorCovPre.at<float>(0) = 10; 
	        kf.errorCovPre.at<float>(7) = 10; 
	        kf.errorCovPre.at<float>(14) = 10;
	        kf.errorCovPre.at<float>(21) = 10;
	        kf.errorCovPre.at<float>(28) = 10; 
	        kf.errorCovPre.at<float>(35) = 10; 

	        /**
             * Set the velocities to be 0, kf.correct() will automatically give them appropriate values
             */
	        state.at<float>(0) = meas.at<float>(0);
	        state.at<float>(1) = meas.at<float>(1);
	        state.at<float>(2) = meas.at<float>(3);
	        state.at<float>(3) = 0;
	        state.at<float>(4) = 0;
	        state.at<float>(5) = 0;

	        kf.statePost = state;
	        
	        found = true;
        }
        else{
        	/**
             * We have some values of state in place
             */
        	kf.correct(meas);
        }
	}else{ 
		/**
         * The buoy was not found in the given
         */
		notFoundCount++;
        if( notFoundCount >= 100 )
        {
            found = false;
        }
	}
}

int main(int argc, char **argv)
{
	/**
     * Pertaining to Kalman filter
     */
    kf = cv::KalmanFilter(stateSize, measSize, contrSize, type);

    // Transition State Matrix A
    // [ 1 0 0 dT   0  0 ]
    // [ 0 1 0  0  dT  0 ]
    // [ 0 0 1  0   0  dT]
    // [ 0 0 0  1   0  0 ]
    // [ 0 0 0  0   1  0 ]
    // [ 0 0 0  0   0  1 ]

    /**
     * video is 25 frames per second
     */
    double dT = 1.0/25.0;
    cv::setIdentity(kf.transitionMatrix);
    kf.transitionMatrix.at<float>(3) = dT;
    kf.transitionMatrix.at<float>(10) = dT;
    kf.transitionMatrix.at<float>(17) = dT;

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 1 0 0 0 ]
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(14) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0   ]
    // [ 0    Ey  0     0     0    0   ]
    // [ 0    0   E_r   0     0    0   ]
    // [ 0    0   0     Ev_x  0    0   ]
    // [ 0    0   0     0     Ev_y 0   ]
    // [ 0    0   0     0     0    Ev_r]
    kf.processNoiseCov.at<float>(0) = 1.0f;
    kf.processNoiseCov.at<float>(7) = 1.0f;
    kf.processNoiseCov.at<float>(14) = 1.0f;
    kf.processNoiseCov.at<float>(21) = 2.0f;
    kf.processNoiseCov.at<float>(28) = 2.0f;
    kf.processNoiseCov.at<float>(35) = 2.0f;

    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

    /**
     * Pertaining to ROS
     */
	ros::init(argc,argv,"kalman");
	ros::NodeHandle n;

	/**
     * Subscribing to measurement data
     */
	ros::Subscriber sub = n.subscribe("data", 1000, dataCallback);

	/**
     * Publilshing predictions
     */
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("prediction", 1000);

	/**
     * Rate of publishing prediction is kept independent of video feed
     */
  	ros::Rate loop_rate(10);

  	while (ros::ok())
  	{
	    std_msgs::String msg;

	    std::stringstream ss;
	    cv::Mat prediction = kf.predict();
	    ss <<prediction.at<float>(0)<<" "<<prediction.at<float>(1)<<" "<<prediction.at<float>(2);
	    msg.data = ss.str();

	    ROS_INFO("Prediction (x,y,r): %s", msg.data.c_str());

	    chatter_pub.publish(msg);

	    ros::spinOnce();

	    loop_rate.sleep();
	    
  	}
	return 0;
}
