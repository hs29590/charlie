#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


using namespace cv;
using namespace std;

class LineErrorCalculator
{
    public:
    LineErrorCalculator()
    {
        m_prevCx = 0;
    }

    ~LineErrorCalculator()
    {
    }
    
    int m_prevCx;

    void imgCallback(const sensor_msgs::ImageConstPtr& msg);

};

void LineErrorCalculator::imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //double start_sec =ros::Time::now().toSec();
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(), 0.25, 0.25);

    Mat hsvImg;
    try
    {
        cv::cvtColor(cv_ptr->image, hsvImg, cv::COLOR_BGR2HSV);
    }
    catch (cv::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Scalar lower_sat = cv::Scalar(220);
    cv::Scalar upper_sat = cv::Scalar(255);

    cv::Mat hsv_channels[3];   //destination array
    cv::split(hsvImg,hsv_channels);//split source  

    //Mat sat_mask;
    Mat sat_mask = Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());
    cv::inRange(hsv_channels[1], lower_sat, upper_sat, sat_mask);

    cv::Scalar lower_yellow = cv::Scalar(0, 120, 120);
    cv::Scalar upper_yellow = cv::Scalar(255, 255, 255);


    //Mat yellow_mask;
    Mat yellow_mask = Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());
    cv::inRange(cv_ptr->image, lower_yellow, upper_yellow, yellow_mask);

    //doing an AND over the results and saving in yellow_mask
    try
    {
        cv::bitwise_and(yellow_mask, sat_mask, yellow_mask);
    }
    catch (cv::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


    cv::GaussianBlur( yellow_mask, yellow_mask, cv::Size( 3, 3 ), 0, 0 );

	Mat kernel;
    kernel = Mat::ones( 3, 3, CV_8U );
   
	// Apply erosion or dilation on the image
	cv::erode(yellow_mask,yellow_mask,kernel);
		cv::imshow("Window", yellow_mask);
		cv::waitKey(3);

	// Draw an example circle on the video stream
   // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
   //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    //cv::imshow("Window", cv_ptr->image);
 //   cv::imshow("Window", outImg);
 //   cv::waitKey(3);

//	CvRect cropRect = cvRect(0, , 100, 300); // ROI in source image
 
    cv::Rect rect((int)cv_ptr->image.size().width/4,
                  0,
                  (int)cv_ptr->image.size().width/2,
                  (int)2*cv_ptr->image.size().height/3);


    Mat roi = yellow_mask(rect);
//	cvSetImageROI(img, cropRect);
//	cvCopy(img, cropImg, NULL); // Copies only crop region
//	cvResetImageROI(img);
	
	cv::Moments mu = cv::moments(roi, false);
	if(mu.m00 > 0)
	{
		int cx = mu.m10/mu.m00;
		int cy = mu.m01/mu.m00;

		cv::circle(cv_ptr->image, cv::Point(cx + (int)cv_ptr->image.size().width/4,cy), 10,  CV_RGB(255,0,0), 4);
		//cv::imshow("Window", cv_ptr->image);
		//cv::imshow("Window", cv_ptr->image);
		//cv::waitKey(3);
	}
	else
	{
        //no moment found
	}
    //double end_sec =ros::Time::now().toSec();
    //std::cout << "time: " << end_sec - start_sec << " sec\n";
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "processImageNode");

    ros::NodeHandle nodeh;

    //create a gui window:
    LineErrorCalculator lineErrorCalculator;
//    ros::Publisher test_pub = nodeh.advertise<std_msgs::String>("/chatter", 1);
    ros::Subscriber img_sub = nodeh.subscribe("/raspicam_node/image_raw", 1, &LineErrorCalculator::imgCallback, &lineErrorCalculator);
    
	ros::spin();
    return 0;

}
