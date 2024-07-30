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

TisCameraManager *cam;

image_transport::CameraPublisher img_raw_pub_;
image_geometry::PinholeCameraModel *pinhole_model_;
camera_info_manager::CameraInfoManager *camera_info_manager_;
sensor_msgs::CameraInfo camera_info;

ros::Publisher image_raw_pub_;
ros::Publisher camera_info_pub_;

std_msgs::Header header_;

std::string format_, camera_n_, camera_sn_, camera_info_url_;
int width_, height_, fps_;
bool tonemapping_, highlight_reduction_;

#define ListPropertiesDebug 0
#define ximagesink_display 0

void ListProperties(gsttcam::TcamCamera &cam)
{
    // Get a list of all supported properties and print it out
    auto properties = cam.get_camera_property_list();
    std::cout << "Properties:" << std::endl;
    for (auto &prop : properties)
    {
        std::cout << prop->to_string() << std::endl;
    }
}

void imageCallback(std::shared_ptr<TisCameraManager::FrameData> data)
{
    if (ros::isShuttingDown())
    {
        ROS_INFO_STREAM("Driver is shutting down. Stopping camera.");
        cam->stop();
        ros::shutdown();
    }
    if (data == nullptr)
    {
        ROS_ERROR("No data received");
        return;
    }
    // pinhole_model_->fromCameraInfo(camera_info_manager_->getCameraInfo());
    auto datainfo = data->get_info();
    // std::cout << datainfo.frame_count << "taken." << std::endl;

    cv::Mat OpenCVImage;
    OpenCVImage.create(datainfo.height, datainfo.width, CV_8UC3);
    memcpy(OpenCVImage.data, data->image_data(), datainfo.bytes_per_pixel * datainfo.height * datainfo.width);
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;

    header_.seq++;                    
    header_.stamp = ros::Time::now(); 
    img_bridge = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::BGR8, OpenCVImage);
    img_bridge.toImageMsg(img_msg);  
    image_raw_pub_.publish(img_msg); 

    if (camera_info_manager_->isCalibrated())
    {
        camera_info.header = img_msg.header;
        camera_info_pub_.publish(camera_info);
    }

    data->release();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tis_camera_node");

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private("~");

    // Load camera info url
    if (nh_private.hasParam("camera_info_url"))
    {
        nh_private.param<std::string>("camera_info_url", camera_info_url_, "");
        camera_info_url_ = "file://" + camera_info_url_;
    }

    // Load parameters
     nh_private.param<std::string>("tis_camera_node/format", format_, "BGR");
    nh_private.param<std::string>("tis_camera_node/camera_name", camera_n_, "tis_front_cam");
    nh_private.param<std::string>("tis_camera_node/camera_serial_number", camera_sn_, "03420356");
    nh_private.param<int>("tis_camera_node/width", width_, 1920);
    nh_private.param<int>("tis_camera_node/height", height_, 1080);
    nh_private.param<int>("tis_camera_node/frame_rate", fps_, 10);
    nh_private.param<bool>("tis_camera_node/tonemapping", tonemapping_, false);
    nh_private.param<bool>("tis_camera_node/highlight_reduction", highlight_reduction_, false);

    header_.frame_id = camera_n_;

    // Initialize the camera info manager
    camera_info_manager_ = new camera_info_manager::CameraInfoManager(nh_, "tis_front_cam", camera_info_url_);

    if (camera_info_manager_->validateURL(camera_info_url_))
    {
        camera_info_manager_->loadCameraInfo(camera_info_url_);
        camera_info = camera_info_manager_->getCameraInfo();
        ROS_INFO_STREAM("Loaded camera calibration from " << camera_info_url_);
    }
    else
    {
        ROS_WARN_STREAM("Camera info at: " << camera_info_url_ << " not found. Using an uncalibrated config.");
    }

    image_raw_pub_ = nh_.advertise<sensor_msgs::Image>("sensor/camera/image_raw", fps_);
    camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("sensor/camera/camera_info", fps_);

    gst_init(&argc, &argv);

    cam = new TisCameraManager(camera_n_, camera_sn_);

    ROS_INFO("Starting Camera");

    // Set false the ximagesink_display, if no live video display is wanted.
    if (ximagesink_display) cam->enable_video_display(gst_element_factory_make("xvimagesink", NULL)); // ximagesink

    // Set a color video format, resolution and frame rate
    cam->set_capture_format(format_, gsttcam::FrameSize{width_, height_}, gsttcam::FrameRate{fps_, 1});

    // Set the parameters for the camera
    cam->set_trigger_mode(TisCameraManager::NONE);
    // cam->set_exposure_gain_auto(true);
    // cam->set_exposure_auto_reference(128);
    // cam->set_exposure_time(30000); // in us
    // cam->set_exposure_auto_reference(true);
    // cam->set_gain(1);

    // Start the camera
    cam->start();

    cam->set_tonemapping_mode(tonemapping_);
    cam->set_highlight_reduction(highlight_reduction_);

    // Set true the ListPropertiesDebug, if you want to see the all camera properties
    if (ListPropertiesDebug) ListProperties(*cam);

    cam->registerCallback(imageCallback);

    ros::spin();

    return 0;
}

/*TODO
- Add some other parameterx ex. exposure, gain, etc.
- Implement the sub header in config file ex. tonemapping -> intensity etc.
- Clean and prepare the interface
- Add the rectified image and publish it
*/