#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


using namespace cv;
using namespace std;

class ImageInfoExtractor
{
    public:
        ImageInfoExtractor()
        {
            m_prev_cx = 0;
            m_show_images = false;
            kernel = Mat::ones( 3, 3, CV_8U );
            check_intersections = true;
            bgr_color_lower[0] = bgr_color_lower[1] = bgr_color_lower[2] = 0;
            hsv_color_lower[0] = hsv_color_lower[1] = hsv_color_lower[2] = 0;
            
            bgr_color_upper[0] = bgr_color_upper[1] = bgr_color_upper[2] = 0;
            hsv_color_upper[0] = hsv_color_upper[1] = hsv_color_upper[2] = 0;
            
            process_scale = 0.25;

            num_white_px = 0;

            //min_object_area = 10*10;
            err.data = -1000.0;
            intersection_err.data = -1000.0;

            no_intersection_count = 0;
            intersection_seen_count = 0;

            no_line_count = 0;
            cx = cy = 0.0;
            erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
            dilateElement = getStructuringElement( MORPH_RECT,Size(6,6));
        }


        ~ImageInfoExtractor()
        {
        }

        float m_prev_cx;
        bool m_show_images;
        bool m_check_left_right_line_err;
        Mat kernel;
        
        int bgr_color_lower[3];
        int bgr_color_upper[3];

        int hsv_color_lower[3];
        int hsv_color_upper[3];

        ros::Publisher err_pub;
        ros::Publisher line_visible_pub;
        ros::Publisher intersection_err_pub;
        ros::Publisher left_image_err_pub;
        ros::Publisher right_image_err_pub;

        float process_scale;

        std_msgs::Float32 err;
        std_msgs::Float32 left_line_err;
        std_msgs::Float32 right_line_err;
        std_msgs::Float32 intersection_err;
        std_msgs::Bool line_visible;

        float cx;
        float cy;

        bool check_intersections;
        int intersection_seen_count;
        int no_intersection_count;
        int num_white_px;
        int no_line_count;

        Mat hsv_mask;
        Mat hsvImg;
        Mat bgr_mask;
	    Mat erodeElement;
        //dilate with larger element so make sure object is nicely visible
    	Mat dilateElement;

        void imgCallback(const sensor_msgs::ImageConstPtr& msg);

};

void ImageInfoExtractor::imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
//    double start_sec =ros::Time::now().toSec();
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

    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(), process_scale, process_scale);

    try
    {
        cv::cvtColor(cv_ptr->image, hsvImg, cv::COLOR_BGR2HSV);
    }
    catch (cv::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    hsv_mask = Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());
    cv::inRange(hsvImg, cv::Scalar(hsv_color_lower[0], hsv_color_lower[1], hsv_color_lower[2]), 
                        cv::Scalar(hsv_color_upper[0], hsv_color_upper[1], hsv_color_upper[2]),
                        hsv_mask);
    
    //cv::erode(hsv_mask,hsv_mask,kernel);
    cv::GaussianBlur( hsv_mask, hsv_mask, cv::Size( 5, 5 ), 0, 0 );

    if(m_show_images)
    {
        cv::imshow("hsv mask", hsv_mask);
        cv::waitKey(3);
    }
    
    bgr_mask = Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());
    cv::inRange(cv_ptr->image, cv::Scalar(bgr_color_lower[0], bgr_color_lower[1], bgr_color_lower[2]), 
                               cv::Scalar(bgr_color_upper[0], bgr_color_upper[1], bgr_color_upper[2]), 
                               bgr_mask);
    if(m_show_images)
    {
        cv::imshow("bgr mask", bgr_mask);
        cv::waitKey(3);
    }

    //doing an AND over the results and saving in yellow_mask
    try
    {
        cv::bitwise_and(bgr_mask, hsv_mask, bgr_mask);
    }
    catch (cv::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

	erode(bgr_mask,bgr_mask,erodeElement);
	erode(bgr_mask,bgr_mask,erodeElement);

	dilate(bgr_mask,bgr_mask,dilateElement);
	dilate(bgr_mask,bgr_mask,dilateElement);

//    cv::GaussianBlur( bgr_mask, bgr_mask, cv::Size( 5, 5 ), 0, 0 );

    // Apply erosion or dilation on the image
  //  cv::erode(bgr_mask,bgr_mask,kernel);

    cv::Rect rect((int)cv_ptr->image.size().width/4,
            0,
            (int)cv_ptr->image.size().width/2,
            (int)2*cv_ptr->image.size().height/3);



    Mat roi = bgr_mask(rect);
    Mat roi_left = roi.clone();
    
    if(m_show_images)
    {
        cv::imshow("Fin Mask", roi);
        cv::waitKey(3);
    }
    
    //Get the moments of the mask to get general direction of the line
    cv::Moments mu = cv::moments(roi, false);
    if(mu.m00 > 500)
    {
        cx = (float)mu.m10/mu.m00;
        cy = (float)mu.m01/mu.m00;

        cv::circle(cv_ptr->image, cv::Point((int)cx + (int)cv_ptr->image.size().width/4,(int)cy), 10,  CV_RGB(255,0,0), 4);
        
        cx = m_prev_cx*0.5 + cx;
        m_prev_cx = cx;

        err.data = (cx - (float)(roi.size().width)/2.0);
        err_pub.publish(err);
        line_visible.data = true;
        no_line_count = 0;
        line_visible_pub.publish(line_visible);
    }
    else
    {
        //no moment found
        no_line_count++;
        if(no_line_count > 5)
            err.data = -1000.0;
        err_pub.publish(err);
        line_visible.data = false;
        line_visible_pub.publish(line_visible);
    }

    if(m_check_left_right_line_err)
    {
        cv::Rect left_rect(0, 0, (int)roi.size().width/2, roi.size().height);
        cv::Rect right_rect((int)roi.size().width/2, 0, (int)roi.size().width/2, roi.size().height);
        roi(left_rect) = 0;
        if(m_show_images)
        {
            cv::imshow("ROI only right", roi);
            cv::waitKey(3);
        }
        mu = cv::moments(roi, false);
        if(mu.m00 > 100)
        {
            cx = (float)mu.m10/mu.m00;
            cy = (float)mu.m01/mu.m00;
            
            cv::circle(cv_ptr->image, cv::Point((int)cx + (int)cv_ptr->image.size().width/4,(int)cy), 10,  CV_RGB(0,0,255), 4);

            right_line_err.data = (cx - (float)(roi.size().width)/2.0);
            right_image_err_pub.publish(right_line_err);
        }
        else
        {
            right_line_err.data = -1000.0;
            right_image_err_pub.publish(right_line_err);
        }
        
        roi_left(right_rect) = 0; // for the left image, make the right pixels black
        if(m_show_images)
        {
            cv::imshow("ROI only left", roi_left);
            cv::waitKey(3);
        }
        mu = cv::moments(roi_left, false);
        if(mu.m00 > 100)
        {
            cx = (float)mu.m10/mu.m00;
            cy = (float)mu.m01/mu.m00;
        
            cv::circle(cv_ptr->image, cv::Point((int)cx + (int)cv_ptr->image.size().width/4,(int)cy), 10,  CV_RGB(0,255,0), 4);

            left_line_err.data = (cx - (float)(roi_left.size().width)/2.0);
            left_image_err_pub.publish(left_line_err);
        }
        else
        {
            left_line_err.data = -1000.0;
            left_image_err_pub.publish(left_line_err);
        }

    }



    if(m_show_images)
    {
        cv::imshow("Fin Img", cv_ptr->image);
        cv::waitKey(3);
    }
    if(check_intersections)
    {
        //checks red intersections
        try
        {
            cv::inRange(hsvImg, cv::Scalar(170,0,0), 
                    cv::Scalar(180,255,255),
                    hsv_mask);

            cv::inRange(hsvImg, cv::Scalar(0,0,0),
                    cv::Scalar(10,255,255),
                    bgr_mask);

            cv::bitwise_or(bgr_mask, hsv_mask, bgr_mask);

   //         cv::erode(bgr_mask,bgr_mask,kernel);
        	erode(bgr_mask,bgr_mask,erodeElement);
        	erode(bgr_mask,bgr_mask,erodeElement);

        	dilate(bgr_mask,bgr_mask,dilateElement);
        	dilate(bgr_mask,bgr_mask,dilateElement);
        }
        catch (cv::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //bgr_mask contains the intersections

        if(m_show_images)
        {
            cv::imshow("intersection mask", bgr_mask);
            cv::waitKey(3);
        }

        num_white_px = cv::countNonZero(bgr_mask);
        if(num_white_px > 30)
        {
            intersection_seen_count++;
            no_intersection_count = 0;
        }
        else
        {
            intersection_seen_count = 0;
            no_intersection_count++;
        }

        if(intersection_seen_count > 5)
        {
            cv::Moments mu = cv::moments(bgr_mask, false);
            if(mu.m00 > 0)
            {
                cx = (float)mu.m10/mu.m00;
                cy = (float)mu.m01/mu.m00;

                intersection_err.data = ((cx - (float)(cv_ptr->image.size().width)/2.0));
                intersection_err_pub.publish(intersection_err);
            }
            else
            {
            //    ROS_ERROR("No Moment found while calculating moment for intersection..");
            intersection_err.data = -1000.0;
            intersection_err_pub.publish(intersection_err);
            }
        }
        else if(no_intersection_count >= 10)
        {
            intersection_err.data = -1000.0;
            intersection_err_pub.publish(intersection_err);
        }
    }
    //double end_sec =ros::Time::now().toSec();
    //ROS_INFO_THROTTLE(5, "Time Taken: %lf sec", end_sec - start_sec);
    //std::cout << end_sec - start_sec << std::endl;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "processImageNode");

    ros::NodeHandle nodeh;

    ImageInfoExtractor imageInfoExtractor;

    nodeh.param("/processImage/b_lower", imageInfoExtractor.bgr_color_lower[0], 0);
    nodeh.param("/processImage/g_lower", imageInfoExtractor.bgr_color_lower[1], 120);
    nodeh.param("/processImage/r_lower", imageInfoExtractor.bgr_color_lower[2], 120);
    nodeh.param("/processImage/b_upper", imageInfoExtractor.bgr_color_upper[0], 255);
    nodeh.param("/processImage/g_upper", imageInfoExtractor.bgr_color_upper[1], 255);
    nodeh.param("/processImage/r_upper", imageInfoExtractor.bgr_color_upper[2], 255);
    
    nodeh.param("/processImage/h_lower", imageInfoExtractor.hsv_color_lower[0], 20);
    nodeh.param("/processImage/s_lower", imageInfoExtractor.hsv_color_lower[1], 220);
    nodeh.param("/processImage/v_lower", imageInfoExtractor.hsv_color_lower[2], 0);
    nodeh.param("/processImage/h_upper", imageInfoExtractor.hsv_color_upper[0], 30);
    nodeh.param("/processImage/s_upper", imageInfoExtractor.hsv_color_upper[1], 255);
    nodeh.param("/processImage/v_upper", imageInfoExtractor.hsv_color_upper[2], 255);
    
    nodeh.param("/processImage/process_scale", imageInfoExtractor.process_scale, 0.25f);

    ROS_INFO("process scale: %f", imageInfoExtractor.process_scale);

    //std::string s;
    //n.param<std::string>("my_param", s, "default_value");
    
    nodeh.param("/processImage/show_images", imageInfoExtractor.m_show_images, true);
    ROS_INFO("Show Images: %d\n", (imageInfoExtractor.m_show_images));

    nodeh.param("/processImage/check_left_right_line_err", imageInfoExtractor.m_check_left_right_line_err, true);
    ROS_INFO("Check Left Right Line Err: %d\n", (imageInfoExtractor.m_check_left_right_line_err));

    nodeh.param("/processImage/check_intersections", imageInfoExtractor.check_intersections, true);


    imageInfoExtractor.err_pub = nodeh.advertise<std_msgs::Float32>("/line_error", 1);
    imageInfoExtractor.line_visible_pub = nodeh.advertise<std_msgs::Bool>("/line_visible", 1);
    imageInfoExtractor.intersection_err_pub = nodeh.advertise<std_msgs::Float32>("/intersection_err", 1);
    imageInfoExtractor.left_image_err_pub = nodeh.advertise<std_msgs::Float32>("/left_line_err", 1);
    imageInfoExtractor.right_image_err_pub = nodeh.advertise<std_msgs::Float32>("/right_line_err", 1);
    ros::Subscriber img_sub = nodeh.subscribe("/raspicam_node/image_raw", 1, &ImageInfoExtractor::imgCallback, &imageInfoExtractor);

    ros::spin();
    return 0;
}
