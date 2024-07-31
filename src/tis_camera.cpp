#include <tis_ros.h>

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
    auto datainfo = data->get_info();
    header_.stamp = ros::Time::now();

    img_raw_.header = header_;
    img_raw_.height = datainfo.height;
    img_raw_.width = datainfo.width;
    img_raw_.step = datainfo.bytes_per_pixel * datainfo.width;
    img_raw_.encoding = sensor_msgs::image_encodings::BGR8;
    img_raw_.data.resize(img_raw_.height * img_raw_.step);
    memcpy(&img_raw_.data[0], data->image_data(), img_raw_.height * img_raw_.step);

    if (image_raw_pub_.getNumSubscribers() > 0)
        image_raw_pub_.publish(img_raw_);

    if (camera_info_manager_->isCalibrated())
    {
        if (camera_info_pub_.getNumSubscribers() > 0)
        {
            camera_info.header = img_raw_.header;
            camera_info_pub_.publish(camera_info);
        }
        if (image_rect_pub_.getNumSubscribers() > 0)
        {
            cv_bridge::CvImagePtr cv_img_raw = cv_bridge::toCvCopy(img_raw_, img_raw_.encoding);
            cv_bridge_img_rect_->header = img_raw_.header;
            cv_bridge_img_rect_->encoding = img_raw_.encoding;
            pinhole_model_->fromCameraInfo(camera_info_manager_->getCameraInfo());
            pinhole_model_->rectifyImage(cv_img_raw->image, cv_bridge_img_rect_->image);
            image_rect_pub_.publish(*cv_bridge_img_rect_);
        }
    }
    header_.seq++;
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
    nh_private.param<bool>("tis_camera_node/debugFlag", debugFlag_, false);
    nh_private.param<bool>("tis_camera_node/propertyFlag", propertyFlag_, false);

    nh_private.param<std::string>("tis_camera_node/camera_name", camera_n_, "tis_front_cam");
    nh_private.param<std::string>("tis_camera_node/camera_serial_number", camera_sn_, "03420356");

    nh_private.param<std::string>("tis_camera_node/format", format_, "BGR");
    nh_private.param<int>("tis_camera_node/width", width_, 1920);
    nh_private.param<int>("tis_camera_node/height", height_, 1080);
    nh_private.param<int>("tis_camera_node/frame_rate", fps_, 10);

    nh_private.param<bool>("tis_camera_node/exposure_auto", exposure_auto_, false);
    nh_private.param<int>("tis_camera_node/set_exposure_time", set_exposure_time_, 10000);
    nh_private.param<bool>("tis_camera_node/exposure_limits", exposure_limits_, false);
    nh_private.param<int>("tis_camera_node/exposure_lower_limit", exposure_lower_limit_, 60);
    nh_private.param<int>("tis_camera_node/exposure_upper_limit", exposure_upper_limit_, 20000);

    nh_private.param<int>("tis_camera_node/exposure_auto_reference", exposure_auto_reference_, 128);

    nh_private.param<int>("tis_camera_node/gain", gain_, 0);
    nh_private.param<int>("tis_camera_node/gamma", gamma_, 0);

    nh_private.param<bool>("tis_camera_node/tonemapping", tonemapping_, false);
    nh_private.param<int>("tis_camera_node/tonemapping_intensity", tonemapping_intensity_, 1.0);
    nh_private.param<double>("tis_camera_node/tonemapping_global_brightness", tonemapping_global_brightness_, 0.5);

    nh_private.param<bool>("tis_camera_node/highlight_reduction", highlight_reduction_, false);

    header_.frame_id = camera_n_;

    // Initialize the camera info manager
    camera_info_manager_ = new camera_info_manager::CameraInfoManager(nh_, "tis_front_cam", camera_info_url_);

    if (camera_info_manager_->validateURL(camera_info_url_))
    {
        camera_info_manager_->loadCameraInfo(camera_info_url_);
        camera_info = camera_info_manager_->getCameraInfo();
        camera_info.header.frame_id = header_.frame_id;
        pinhole_model_ = new image_geometry::PinholeCameraModel();
        cv_bridge_img_rect_ = new cv_bridge::CvImage();
        ROS_INFO_STREAM("Loaded camera calibration from " << camera_info_url_);
    }
    else
    {
        ROS_WARN_STREAM("Camera info at: " << camera_info_url_ << " not found. Using an uncalibrated config.");
    }

    image_raw_pub_ = nh_.advertise<sensor_msgs::Image>("sensor/camera/image_raw", 1);
    camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("sensor/camera/camera_info", 1);
    image_rect_pub_ = nh_.advertise<sensor_msgs::Image>("sensor/camera/image_rect", 1);

    gst_init(&argc, &argv);

    cam = new TisCameraManager(camera_n_, camera_sn_);

    ROS_INFO("Starting Camera");

    // Set false the ximagesink_display, if no live video display is wanted.
    if (debugFlag_)
        cam->enable_video_display(gst_element_factory_make("xvimagesink", NULL)); // ximagesink

    // Set a color video format, resolution and frame rate
    cam->set_capture_format(format_, gsttcam::FrameSize{width_, height_}, gsttcam::FrameRate{fps_, 1});

    // Set the parameters for the camera
    cam->set_trigger_mode(TisCameraManager::NONE);
    if (exposure_auto_) cam->set_exposure_gain_auto(true);
    if (!exposure_auto_) cam->set_exposure_time(set_exposure_time_);
    if (exposure_limits_) cam->set_exposure_limits(exposure_limits_, exposure_lower_limit_, exposure_upper_limit_);
    if (exposure_auto_reference_) cam->set_exposure_auto_reference(exposure_auto_reference_);

    // Start the camera
    cam->start();

    if (tonemapping_) 
    {
        cam->set_tonemapping_mode(tonemapping_);
        cam->set_tonemapping_param(tonemapping_intensity_, tonemapping_global_brightness_);
    }
    if (highlight_reduction_) cam->set_highlight_reduction(highlight_reduction_);

    // Set true the ListPropertiesDebug, if you want to see the all camera properties
    if (propertyFlag_)
        ListProperties(*cam);

    cam->registerCallback(imageCallback);

    ros::spin();

    return 0;
}

/*TODO
- Add some other parameterx ex. exposure, gain, etc.
- Clean and prepare the interface
*/