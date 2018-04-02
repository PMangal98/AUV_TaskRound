/**
 * Tracking using Kalman Filter
 */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <vector>

using namespace std;


int main()
{
    /**
     * frame to hold the video frame
     */
    cv::Mat frame;

    /**
     * Kalman Filter
     */
    int stateSize = 6;  //size of state
    int measSize = 3;   //size of measurement
    int contrSize = 0;  //size of control

    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

    cv::Mat state(stateSize, 1, type);      // [x,y,r,v_x,v_y,v_r]
    cv::Mat meas(measSize, 1, type);        // [z_x,z_y,z_r]
    cv::Mat procNoise(stateSize, 1, type);  // [E_x,E_y,E_r,E_v_x,E_v_y,E_v_r]

    // Transition State Matrix A
    // [ 1 0 0 dT   0  0 ]
    // [ 0 1 0  0  dT  0 ]
    // [ 0 0 1  0   0  dT]
    // [ 0 0 0  1   0  0 ]
    // [ 0 0 0  0   1  0 ]
    // [ 0 0 0  0   0  1 ]
    double dT = 1.0/25.0;//video is 25 frames per second
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

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

    /**
     * Camera capture
     */
    cv::VideoCapture cap("/home/param/Desktop/OpenCV1/Task Buoy/frontcam.avi");

    cout << "\nHit 'q' to exit...\n";
    char ch = 0;

    /**
     * Bool pertaining to whether the buoy was found previously
     */
    bool found = false;

    /**
     * No of frame since which the buoy was lost
     */
    int notFoundCount = 0;

    while (ch != 'q' && ch != 'Q')
    {
        /**
         * Acquiring Frame
         */
        cap >> frame;

        cv::Mat res;
        frame.copyTo( res );

        /**
         * If buoy was found at an earlier frame, draw a prediction for where the buoy would be in this frame
         */
        if (found)
        {
            /**
             * Getting the prediction
             */
            state = kf.predict();
            cout << "State post:" << endl << state << endl

            cv::Point center;
            center.x = state.at<float>(0);
            center.y = state.at<float>(1);
            double r = state.at<float>(2);

            /**
             * Drawing a circle around the prediction area
             */
            cv::circle(res, center, 2, CV_RGB(255,0,0), -1);
        }

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
         * Separating out hue and saturation channels
         */
        cv::Mat frameSat;
        std::vector<cv::Mat> channels;
        split(frmHsv,channels);
        channels[0].copyTo(frmHsv);
        channels[1].copyTo(frameSat);

        /**
         * Range of Red Hue
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
         * Saturation range to separate out relection of buoy
         */
        cv::Scalar sat_l(90);
        cv::Scalar sat_h(255);
        cv::Mat actual_sat;
        inRange(frameSat, sat_l, sat_h, actual_sat);

        /**
         * Binarized Image
         */
        cv::Mat rangeRes = (red_hue_lower | red_hue_upper)&actual_sat;

        /**
         * Improving the result
         */
        cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 5);
        cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 5);

        /**
         * Thresholding viewing
         */
        cv::imshow("Threshold", rangeRes);

        /**
         * Contours Detection
         */
        vector<vector<cv::Point> > contours;
        cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

        /**
         * Vector to hold contours of buoys
         */
        vector<vector<cv::Point> > buoys;
        vector<cv::Rect> buoyBox; 

        /**
         * Finding Contour with max area, if there are ant contours
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

        if(index!=-1){
            buoys.push_back(contours[index]);
            buoyBox.push_back(cv::boundingRect(contours[index]));
        }


        /**
         * Draw Contours of measured value if a buoy was detected
         */
        if((int)buoys.size()==1)
        {
            cv::drawContours(res, buoys, 0, CV_RGB(20,150,20), 5);
            cv::rectangle(res, buoyBox[0], CV_RGB(0,255,0), 2);

            cv::Point center;
            center.x = buoyBox[0].x + buoyBox[0].width / 2;
            center.y = buoyBox[0].y + buoyBox[0].height / 2;
            cv::circle(res, center, 2, CV_RGB(20,150,20), -1);

            stringstream sstr;
            sstr << "(" << center.x << "," << center.y << ")";
            cv::putText(res, sstr.str(),
                        cv::Point(center.x + 3, center.y - 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
        }

        /**
         * Performing Kalman update
         */
        if (buoys.size() == 0)
        {
            /**
             * If notFoundCount is above 100, we have lost the object
             */
            notFoundCount++;
            cout << "notFoundCount:" << notFoundCount << endl;
            if( notFoundCount >= 100 )
            {
                found = false;
            }
        }
        else
        {
            /**
             * Buoy was found so we set notFoundCount = 0
             */
            notFoundCount = 0;

            /**
             * Making measurements
             */
            meas.at<float>(0) = buoyBox[0].x + buoyBox[0].width / 2;
            meas.at<float>(1) = buoyBox[0].y + buoyBox[0].height / 2;
            meas.at<float>(2) = sqrt((float)buoyBox[0].width*(float)buoyBox[0].width + (float)buoyBox[0].height*(float)buoyBox[0].height)/2;

            if (!found) 
            {
                /**
                 * First Detection
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
                state.at<float>(2) = meas.at<float>(2);
                state.at<float>(3) = 0;
                state.at<float>(4) = 0;
                state.at<float>(5) = 0;

                kf.statePost = state;
                
                found = true;
            }
            else
            {
                /**
                 * Kalman Correction
                 */
                kf.correct(meas);
            }

            cout << "Measure matrix:" << endl << meas << endl;
        }

        cv::imshow("Tracking", res);

        ch = cv::waitKey(0);
    }

    return 0;
}
