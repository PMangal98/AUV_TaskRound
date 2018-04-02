/**
  *	Code to detect a red buoy framewise.
  */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include<bits/stdc++.h>

using namespace std;
using namespace cv;


int main(){
	/**
 	 * Initializing Frame
 	 */
	VideoCapture vid("/home/param/Desktop/OpenCV1/Task Buoy/frontcam.avi");
	//VideoCapture vid("bouyvideo_ml.avi");

	while(true){
		/**
 	 	 * Acquiring a frame
 	 	 */
		cv::Mat frame;
		vid>>frame;

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
 	 	 * Separating hue and saturation channels
 	 	 */
	    cv::Mat frameSat;
	    std::vector<cv::Mat> channels;
	    split(frmHsv,channels);
	    channels[0].copyTo(frmHsv);
	    channels[1].copyTo(frameSat);

	    /**
 	 	 * Red Range
 	 	 */
	    int redRange_upper = 20;
	    int redRange_lower = 10;

	    /**
 	 	 * One Range for red hue
 	 	 */
	    cv::Scalar hsv_upper_l(180-redRange_upper);
	    cv::Scalar hsv_upper_h(180);
	    cv::Mat red_hue_upper;
	    inRange(frmHsv, hsv_upper_l, hsv_upper_h, red_hue_upper);

	    /**
 	 	 * Another Range for red hue
 	 	 */
	    cv::Scalar hsv_lower_l(0);
	    cv::Scalar hsv_lower_h(0+redRange_lower);
	    cv::Mat red_hue_lower;
	    inRange(frmHsv, hsv_lower_l, hsv_lower_h, red_hue_lower);

	    /**
 	 	 * Saturation value to help separate out reflection
 	 	 */
	    cv::Scalar sat_l(20);
	    cv::Scalar sat_h(150);
	    cv::Mat actual_sat;
	    inRange(frameSat, sat_l, sat_h, actual_sat);

	    /**
 	 	 * Binarization
 	 	 */
	    cv::Mat rangeRes = (red_hue_lower | red_hue_upper)&actual_sat;
	    
	    /**
 	 	 * Dilating binarizing image 
 	 	 */
	    cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 5);
	    
	    /**
 	 	 * Contour Detection
 	 	 */
	    std::vector<std::vector<cv::Point> > contours;
	    cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

	    /**
 	 	 * Making a vector of at most one element, that is the buoy
 	 	 */
	    std::vector<std::vector<cv::Point> > buoys;
	    std::vector<cv::Rect> buoyBox; 

	    /**
 	 	 * Finding contour with max area
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

	    /**
 	 	 * DrawContour if buoy was detected
 	 	 */
	    if(index!=-1)
	    {
	    	buoys.push_back(contours[index]);
	    	cv::drawContours(frame, buoys, 0, CV_RGB(20,150,20), 10);
	    }

	    /**
 	 	 * Imshow
 	 	 */
	    cv::imshow("Detection", frame);
	    
	    cv::waitKey(40);
	}
	return 0;
}
