#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <string>

#include "tiscamera_interface/tiscamera_interface.hpp"

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

TisCameraManager *cam = nullptr; // ?

image_transport::CameraPublisher img_raw_pub_;
image_geometry::PinholeCameraModel *pinhole_model_;
camera_info_manager::CameraInfoManager *camera_info_manager_;
sensor_msgs::CameraInfo camera_info;

ros::Publisher image_raw_pub_;
ros::Publisher camera_info_pub_;

std_msgs::Header header_;

std::string camera_n_, camera_sn_, camera_info_url_;
int fps_;
