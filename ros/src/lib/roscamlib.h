/**
 * RosCamera Library
 * Сервисная библиотека
 * \version 1.01
 * \date 24.07.2014
 * LP 24.07.2014
*/

#ifndef _ROSCAMLIB_H_
#define _ROSCAMLIB_H_

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <sstream>

#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//----------------------------------------------------------
//
//----------------------------------------------------------

class TRosCam
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  cv_bridge::CvImagePtr cv_ptr;
  IplImage wrcamera;

public:
  /// topicname: "/camera/image_raw"
  TRosCam(const char* topicname) : it_(nh_)  
  {
    image_sub_ = it_.subscribe(topicname, 1, &TRosCam::imageCb, this);
    CameraPtr = NULL;
  };

  ~TRosCam()  { };

  void imageCb(const sensor_msgs::ImageConstPtr& original_image)
  {
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    try
    {
      cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("tutorialROSOpenCV::roscamlib::cv_bridge exception: %s", e.what());
      return;
    }
    wrcamera = cv_ptr->image;
    CameraPtr = &wrcamera;
  };
  
  IplImage *CameraPtr;
};

#endif
