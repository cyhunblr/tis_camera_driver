#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <string>

#include <tiscamera_interface/tiscamera_interface.hpp>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <image_geometry/pinhole_camera_model.h>
#include <camera_info_manager/camera_info_manager.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

TisCameraManager *cam;

ros::Publisher image_raw_pub_;
ros::Publisher camera_info_pub_;
ros::Publisher image_rect_pub_;

std_msgs::Header header_;
image_geometry::PinholeCameraModel *pinhole_model_;
camera_info_manager::CameraInfoManager *camera_info_manager_;
sensor_msgs::Image img_raw_;
sensor_msgs::CameraInfo camera_info;
cv_bridge::CvImage* cv_bridge_img_rect_;

std::string format_, camera_n_, camera_sn_, camera_info_url_;
int width_, height_, fps_, set_exposure_time_, exposure_lower_limit_, exposure_upper_limit_, exposure_auto_reference_, gain_, gamma_, tonemapping_intensity_;
double tonemapping_global_brightness_;
bool exposure_auto_, exposure_limits_, tonemapping_, highlight_reduction_, debugFlag_, propertyFlag_;

#define ListPropertiesDebug 1
#define ximagesink_display 0
