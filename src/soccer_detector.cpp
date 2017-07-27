#include <math.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_breidge/cv_bridge.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <soccer_bot/HueConfig.h>

cv::Moments moments;
cv::Mat src, hsv, mask;
cv::Mat dst, cdst;
cv::Point center_of_mass;
 
image_transport::Publisher user_image_pub;
image_transport::Subscriber raw_image_sub;
 
ros::Publisher line_error_pub;
ros::Subscriber sonar_sub; 
std_msgs::Float32 error_msg;

int Sonar_thresh = 10;
double line_center = 0;
int intersections = 0;
int hue_lower, hue_upper, sat_lower, sat_upper, value_lower, value_upper;
bool first_reconfig = true;


bool ball_captured = false;
bool ball_detected = false
bool goal
// STATES
int LOOKING_FOR_BALL = 0;
int CHASING_BALL = 1;
int LOOKING_FOR_GOAL = 2;
int CHASING_GOAL = 3;
int GOAL = 4; 
int CUR_STATE;

void sonarCallback(const std_msgs::Int16& msg)
{ 
	if (msg.data <= 5)
	{
		CUR_STATE = LOOKING_FOR_GOAL
	}else{
		CUR_STATE = LOOKING_FOR_BALL
}
}	
 
void imageCallback(const sensor_msgs::ImageConstPtr& msg)

	switch (CUR_STATE); {
		case LOOKING_FOR_BALL:
			hue_lower=2
			hue_upper=52
			sat_lower=64
			sat_upper=255
			value_lower=182
			value_upper=25		

		case CHASING_BALL:
			hue_lower=2
        	hue_upper=52
        	sat_lower=64
        	sat_upper=255
        	value_lower=182
        	value_up per=255

		case LOOKING_FOR_GOAL:
			hue_lower = 8
			hue_upper = 13
			sat_lower = 0
			sat_upper = 255
			value_lower=0
			value_upper= 255 ;
		case CHASING_GOAL:
			 hue_lower = 8
            hue_upper = 13
            sat_lower = 0
            sat_upper = 255
            value_lower=0
            value_upper= 255 ;
		case GOAL:
			bool goal = true;	

//lower and upper thresholds for pixels on the line
    cv::Scalar lower_thresh(hue_lower, sat_lower, value_lower);
    cv::Scalar upper_thresh(hue_upper, sat_upper, value_upper); 

    src = cv_bridge::toCvShare(msg, "bgr8")->image;
 
    //Convert the image to HSV
    cv::cvtColor(src, hsv, CV_BGR2HSV);
 
    //lower and upper thresholds for pixels on the line
    cv::Scalar lower_thresh(hue_lower, sat_lower, value_lower);
    cv::Scalar upper_thresh(hue_upper, sat_upper, value_upper);
 
    //Create a mask with only white pixels
    cv::inRange(hsv, lower_thresh, upper_thresh, mask);
 
    //Calculate moments of mask
    moments = cv::moments(mask, true);
 
    //Compute Center of Mass
    center_of_mass.x = moments.m10 / moments.m00;
    center_of_mass.y = moments.m01 / moments.m00;
 

    ROS_INFO("moments.m00 is %f", moments.m00);
    // send a secial error if the mass is below a threshold:
    if(moments.m00 > 500){
        //Calculate Normalized Error
        error_msg.data = center_of_mass.x - src.cols/2;
    	if CUR_STATE = 

	}else{
        // special oerror message
        error_msg.data = 12345;
    }
    line_error_pub.publish(error_msg);
 
    //Conertthe image back to BGR
    cv::cvtColor(mask, dst, CV_GRAY2BGR);
 
    //Plot the center of mass
    cv::circle(dst, center_of_mass, 5, cv::Scalar(0,0,255), -1);
 
    sensor_msgs::ImagePtr msg;
 
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
 
    user_image_pub.publish(msg);

	if (ball_detected = false)
	{
	//set velocity
	}
  }
}
 
void reconfigure_callback(soccer_bot::HueConfig &config, uint32_t level)
{
  if (first_reconfig)
  {
    first_reconfig = false;
    return;
  }

  hue_lower = config.hue_lower;
  hue_upper = config.hue_upper;
  sat_lower = config.sat_lower;
  sat_upper = config.sat_upper;
  value_lower = config.value_lower;
  value_upper = config.value_upper;
}


int main(int argc, char **argv)
{
  ROS_INFO("Starting soccer_detector");
  ros::init(argc, argv, "line_detector");
  ros::NodeHandle nh;
  sonar_sub = nh.subscribe<std_msgs::Int16>("/arduino/sonar_2", 10, sonarCallback);
  


  nh.param<int>("hue_lower", hue_lower, 80);
  nh.param<int>("hue_upper", hue_upper, 150);
  nh.param<int>("sat_lower", sat_lower, 20);
  nh.param<int>("sat_upper", sat_upper, 255);
  nh.param<int>("value_lower", value_lower, 20);
  nh.param<int>("value_upper", value_upper, 255);

  image_transport::ImageTransport it(nh);

  dynamic_reconfigure::Server<soccer_bot::HueConfig> config_server;
  dynamic_reconfigure::Server<soccer_bot::HueConfig>::CallbackType f;
  f = boost::bind(&reconfigure_callback, _1, _2);
  config_server.setCallback(f);

 
  //advertise the topic that outputs line error
  line_error_pub = nh.advertise<std_msgs::Float32>("/line_error", 10);
 
  //advertise the topic with our processed image
  user_image_pub = it.advertise("/user/image1", 10);
 
  //subscribe to the raw usb camera image
  raw_image_sub = it.subscribe("/usb_cam/image_raw", 10, imageCallback);
 
  ros::spin();
}
