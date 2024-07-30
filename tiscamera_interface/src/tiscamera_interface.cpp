/*
 * @Author: Cheng Huimin 
 * @Date: 2019-09-13 14:33:01 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2020-06-27 19:32:09
 */

#include "tiscamera_interface.hpp"
#include "tiscamera_interface/tcam_statistics_meta.h"

#include <chrono>
#include <iostream>

#include <cassert>
#include <cstring>

using namespace gsttcam;

TisCameraManager::TisCameraManager(const std::string topic_ns, const std::string serial) : TcamCamera(serial)
{
    // initialise the frame pointer
    frame = std::make_shared<FrameData>(topic_ns, serial);

    // frame.data->frame_count = 0;
    assert(!frame->initialised());

    prop_trigger_mode = get_property("Trigger Mode");
    prop_trigger_polarity = get_property("Trigger Polarity");
    //prop_trigger_exposure_mode = get_property("Trigger Exposure Mode");
    //prop_imx_low_latency_mode = get_property("IMX Low-Latency Mode");

    prop_exposure_mode = get_property("Exposure Auto");
    prop_exposure_time = get_property("Exposure Time (us)");
    prop_gain_mode = get_property("Gain Auto");
    prop_gain = get_property("Gain");

    // tone mapping properties can only be set after camera is started

    std::cout << "Tiscamera initialised" << std::endl;
}

TisCameraManager::~TisCameraManager()
{

    // make sure all threads are shutdown, mutex released
    // if(frame.data.image_data)
    //     delete frame.data.image_data;

    std::cout << "Tiscamera Destructor()" << std::endl;
}

bool TisCameraManager::set_trigger_mode(TisCameraManager::TriggerMode value)
{

    // Always use trigger exposure mode as Frame start
    // if (!prop_trigger_exposure_mode->set((*this), "Frame Start"))
    // {
    //     throw std::runtime_error("set trigger exposure mode FAILED");
    // }

    if (value == NONE)
    {
        std::cout << "setting trigger mode to NONE" << std::endl;

        if (!prop_trigger_mode->set((*this), (int)false))
        {
            throw std::runtime_error("set trigger mode to NONE FAILED");
        }
            
    }else
    {
        std::cout << "setting trigger mode to Enabled" << std::endl;

        if (!prop_trigger_mode->set((*this), (int)true))
        {
            throw std::runtime_error("set trigger mode to True FAILED");
        }

        if (value == TRIGGER_RISING_EDGE)
        {
            std::cout << "setting trigger polarity to RISING EDGE" << std::endl;

            if (!prop_trigger_polarity->set((*this), "Rising Edge"))
            {
                throw std::runtime_error("set trigger polarity FAILED");
            }
        }else if (value == TRIGGER_FALLING_EDGE)
        {
            std::cout << "setting trigger polarity to FAILLING EDGE" << std::endl;

            if (!prop_trigger_polarity->set((*this), "Falling Edge"))
            {
                throw std::runtime_error("set trigger polarity FAILED");
            }
        }

        
    }

    // try
    // {
    //     prop_trigger_mode = get_property("Trigger Mode");
    // }
    // catch(std::exception &ex)    
    // {
    //     printf("Error %s : %s\n",ex.what(), "Trigger Mode");
    //     return false;
    // }

    // std::cout  << prop_trigger_mode->to_string() << std::endl;

    return true;
}

TisCameraManager::TriggerMode TisCameraManager::get_trigger_mode()
{
    int trigger_mode;
    if (!prop_trigger_mode->get((*this), trigger_mode))
    {
        throw std::runtime_error("get trigger mode FAILED");
    }

    if (trigger_mode == 0)
        return TriggerMode::NONE;
    
    std::string trigger_polarity;
    if (!prop_trigger_polarity->get((*this), trigger_polarity))
    {
        throw std::runtime_error("get trigger polarity FAILED");
    }

    if (trigger_polarity == "Rising Edge")
        return TriggerMode::TRIGGER_RISING_EDGE;
    else
        return TriggerMode::TRIGGER_FALLING_EDGE;

}

bool TisCameraManager::set_imx_low_latency_mode(bool value)
{
    std::cout << "setting imx low latency mode to " << value << std::endl;
    if (!prop_imx_low_latency_mode->set((*this), value))
    {
        throw std::runtime_error("set imx low latency mode FAILED");
    }
    return true;
}

bool TisCameraManager::set_exposure_gain_auto(bool value)
{
    std::cout << "setting exposure & gain mode to " << value << std::endl;

    if (!prop_exposure_mode->set((*this), value))
    {
        throw std::runtime_error("set exposure mode FAILED");
    }

    if (!prop_gain_mode->set((*this), value))
    {
        throw std::runtime_error("set gain mode FAILED");
    }

    return true;
}

bool TisCameraManager::set_exposure_time(int value)
{
    std::cout << "setting exposure time to " << value << std::endl;

    if (!prop_exposure_time->set((*this), value))
    {
        throw std::runtime_error("set exposure time FAILED");
    }
    return true;
}

bool TisCameraManager::set_exposure_limits(bool auto_upper, int lower, int upper)
{
    std::cout << "set auto upper exposure limit to " << auto_upper << std::endl;
    std::cout << "setting exposure limits to " << lower << " to " << upper << " us" << std::endl;

    prop_auto_exposure_upper_limit = get_property("Exposure Auto Upper Limit Auto");

    if (!prop_auto_exposure_upper_limit->set((*this), auto_upper))
    {
        throw std::runtime_error("set auto upper exposure limit mode FAILED");
    }

    prop_lower_limit_exposure = get_property("Exposure Auto Lower Limit");
    prop_upper_limit_exposure = get_property("Exposure Auto Upper Limit");

    if (!prop_lower_limit_exposure->set((*this), lower))
    {
        throw std::runtime_error("set exposure lower limit FAILED");
    }

    if (!prop_upper_limit_exposure->set((*this), upper))
    {
        throw std::runtime_error("set exposure upper limit FAILED");
    }

    return true;
}

bool TisCameraManager::set_exposure_auto_reference(int value)
{
    std::cout << "setting exposure auto reference to " << value << std::endl;
    prop_exposure_auto_reference = get_property("Exposure Auto Reference");

    if (!prop_exposure_auto_reference->set((*this), value))
    {
        throw std::runtime_error("set exposure exposure auto reference FAILED");
    }

    return true;
}

bool TisCameraManager::set_gain(int value)
{
    std::cout << "setting gain to " << value << std::endl;

    if (!prop_gain->set((*this), value))
    {
        throw std::runtime_error("set gain FAILED");
    }
    return true;
}

bool TisCameraManager::set_gain_limits(int lower, int upper)
{
    std::cout << "setting gain limits to " << lower << " to " << upper << " us" << std::endl;

    prop_lower_limit_gain = get_property("Gain Auto Lower Limit");
    prop_upper_limit_gain = get_property("Gain Auto Upper Limit");

    if (!prop_lower_limit_gain->set((*this), lower))
    {
        throw std::runtime_error("set gain lower limit FAILED");
    }

    if (!prop_upper_limit_gain->set((*this), upper))
    {
        throw std::runtime_error("set gain upper limit FAILED");
    }

    return true;
}

bool TisCameraManager::set_gamma(float value)
{
    std::cout << "setting gamma to " << value << std::endl;

    prop_gamma = get_property("Gamma");

    if (!prop_gamma->set((*this), value))
    {
        throw std::runtime_error("set gamma FAILED");
    }
    return true;
}

bool TisCameraManager::set_tonemapping_mode(bool value)
{

    // std::cout << "Obtaining properties for Tonemapping" << std::endl;
    prop_tonemapping_mode = get_property("Tonemapping");

    std::cout << "setting tonemapping mode to " << value << std::endl;

    if (!prop_tonemapping_mode->set((*this), value))
    {
        throw std::runtime_error("set tonemapping mode FAILED");
        return false;
    }
    return true;
}

bool TisCameraManager::set_tonemapping_param(float intensity, float global_brightness)
{
    prop_tonemapping_intensity = get_property("Tonemapping Intensity");
    prop_tonemapping_global_brightness = get_property("Tonemapping Global Brightness");

    std::cout << "setting tonemapping intensity to " << intensity << std::endl;
    std::cout << "setting tonemapping global brightness to " << global_brightness << std::endl;

    if (!prop_tonemapping_intensity->set((*this), intensity))
    {
        throw std::runtime_error("set tonemapping intensity FAILED");
        return false;
    }

    if (!prop_tonemapping_global_brightness->set((*this), global_brightness))
    {
        throw std::runtime_error("set global brightness mode FAILED");
        return false;
    }

    return true;
}


bool TisCameraManager::set_highlight_reduction(bool value)
{
    prop_highlight_reduction = get_property("Highlight Reduction");
    if (!prop_highlight_reduction->set((*this), value))
    {
        throw std::runtime_error("set Highlight Reduction FAILED");
        return false;
    }

    return true;
}

void TisCameraManager::set_capture_format(std::string format, FrameSize size, FrameRate framerate)
{
    TcamCamera::set_capture_format(format, size, framerate);
}


bool TisCameraManager::start()
{
    std::cout << frame->get_info().topic_ns <<": starting camera..." << std::endl;

    set_new_frame_callback(std::bind(&TisCameraManager::setFrame, this, std::placeholders::_1, std::placeholders::_2), &frame);

    return TcamCamera::start();
}

bool TisCameraManager::stop()
{
    std::cout << frame->get_info().topic_ns <<": stopping camera..." << std::endl;

    return TcamCamera::stop();
}


GstFlowReturn TisCameraManager::setFrame(GstAppSink *appsink, gpointer data)
{
    (void)data; // unused

    // no callback yet, just return
    if (!_cblist_camera.size())
        return GST_FLOW_OK;
    
    if(frame->try_lock())
    {
        // check if the old data is still "in use"
        // if in use, create a totally new buffer in the heap
        

        if (frame->initialised()){
            auto info = frame->get_info();
            std::cout << "TisCameraManager: Creating new data buffer for camera " << info.topic_ns <<", as the previous one still in use" << std::endl;
            frame->unlock(); // unlock it, as we are not using it below this
            frame = std::make_shared<FrameData>(info.topic_ns, info.camera_sn);
        }

        // from here, we have ensure the data is not initialised, mean can be over-writted safely

        auto info = frame->get_info();

        const uint old_buffer_size = info.width * info.height * info.bytes_per_pixel;

        // obtain buffer location
        GstSample *sample = gst_app_sink_pull_sample(appsink);
        GstBuffer *buffer = gst_sample_get_buffer(sample);

        //// THIS IS NULL
        // const GstStructure * sample_info =  gst_sample_get_info(sample);

        // obtain caps associated
        const GstCaps  *caps = gst_sample_get_caps(sample);
        const GstStructure * caps_structure = gst_caps_get_structure(caps, 0);

        info.pixel_format = gst_structure_get_string(caps_structure, "format");

        if (info.pixel_format == "GRAY16_LE")
            info.bytes_per_pixel = 2;
        else if (info.pixel_format == "GRAY_8")
            info.bytes_per_pixel = 1;
        else if (info.pixel_format == "BGR")
            info.bytes_per_pixel = 3;
        else
            throw std::runtime_error("Unkown pixel format " + info.pixel_format);

        gst_structure_get_int(caps_structure, "width", &info.width);
        gst_structure_get_int(caps_structure, "height", &info.height);

        // std::cout  << "frame_width " << frame_width << ", frame_height " << frame_height << ", bytes_per_pixel " << bytes_per_pixel << std::endl;

        
        // DEBUG: BUFFER metadata
        // https://thiblahute.github.io/GStreamer-doc/design/meta.html?gi-language=c
        // https://developer.gnome.org/gstreamer/stable/gstreamer-GstBuffer.html#gst-buffer-get-meta
        
        // Obtain metadata from Tiscamera specifics

        GstMeta* meta = gst_buffer_get_meta(buffer, g_type_from_name("TcamStatisticsMetaApi"));
        if (meta)
        {
            // printf("We have meta\n");
            info.meta_api = meta->info->api;
        }
        else
        {
            g_warning("No meta data available\n");
        }

        // if(!frame.data.initialised){
        //     int meta_count = 0;
        //     GstMeta *current;
        //     gpointer state = NULL;

        //     current = gst_buffer_iterate_meta(buffer, &state);

        //     while(current != nullptr){
        //         printf("gst_meta_api_type_get_tags = %d\n" , gst_meta_api_type_get_tags(current->info->api));
        //         const char* name = g_type_name (current->info->type);
        //         printf("g_type_name_%d = %s\n", meta_count, name);

        //         if(!strcmp(name, "TcamStatisticsMeta")){ // changed string to TcamStatisticsMetaApi in version tiscamera 0.12
        //             frame.data.meta_api = current->info->api; 
        //             std::cout  << "We have meta for Tiscamera!" << std::endl;
        //         }

        //         current = gst_buffer_iterate_meta(buffer, &state);
        //         meta_count++;
        //     }

        //     std::cout  << "Found " << meta_count << " metadata entries in the buffer frame" << std::endl;
        // }

        // const GstMetaInfo * meta_info = gst_meta_get_info("TcamStatisticsMeta");

        if (info.meta_api > 0){
            TcamStatisticsMeta *meta = (TcamStatisticsMeta *) gst_buffer_get_meta(buffer, info.meta_api);

            assert(meta);

            gst_structure_get_uint64(meta->structure, "frame_count", &info.frame_count);
            gst_structure_get_uint64(meta->structure, "frames_dropped", &info.frames_dropped); 
            // this is a monotonic clock, but not realtime
            gst_structure_get_uint64(meta->structure, "capture_time_ns", &info.capture_time_ns);
            gst_structure_get_double(meta->structure, "framerate", &info.framerate);
            gst_structure_get_boolean(meta->structure, "is_damaged", &info.is_damaged);
        }
        


        //// Obtain buffer info
        GstMapInfo gstinfo;
        gst_buffer_map(buffer, &gstinfo, GST_MAP_READ); // read-only

        // buffer should match caps
        assert( (int)gstinfo.size == info.width * info.height * info.bytes_per_pixel);

        assert(gstinfo.data);

        if (frame->image_data() == nullptr)
            frame->allocate(gstinfo.size);
        else if (old_buffer_size!= gstinfo.size)
        {
            std::cout << "Detected change of buffer size from " << old_buffer_size << " to  " << gstinfo.size << std::endl;
            frame->delete_data();
            frame->allocate(gstinfo.size);
        }

        // we make a deep copy of the image from the GStreamer buffer to our local buffer
        // This buffer is reused everytime
        frame->write_data(gstinfo.data, info.width * info.height * info.bytes_per_pixel);
        
        // Calling Unref is important!
        gst_buffer_unmap (buffer, &gstinfo);
        gst_sample_unref(sample);

        

        //std::cout << info.topic_ns << " " << info.capture_time_ns  << " frame " << info.frame_count << std::endl;

        frame->set_info(info);
        frame->unlock();
        frame->con.notify_all();

        // call callbacks outside the mutex
        if (_cblist_camera.size())
        {
            for (auto& cb : _cblist_camera)
                cb(frame);
        }
    }
    else
    {
        auto info = frame->get_info();
        std::cerr << info.topic_ns << "Missed Frame " << info.frame_count << std::endl;
    }

    return GST_FLOW_OK;
}

// should run on a separate thread
void TisCameraManager::processFrame()
{

    std::cout << frame->get_info().topic_ns <<  ": process frame loop starts..." << std::endl;
    while (is_playing)
    {
        std::unique_lock<std::recursive_mutex> lk(frame->mtx); // this call also locks the thread, with blocking behaviour
        auto ret = frame->con.wait_for(lk,std::chrono::seconds(2)); // with ~0.03ms delay, lock reacquired

        if (ret == std::cv_status::timeout ){
            std::cerr << frame->get_info().topic_ns << ": Wait timeout for new frame arrival..." << std::endl;
            continue;
        }

        // frame.data.image_data = nullptr; // make the next frame to take a new memory space
    }

    std::cout << frame->get_info().topic_ns <<  ": process frame loop terminates..." << std::endl;
}

void TisCameraManager::registerCallback(TisCameraManager::callbackCamera cb)
{
    _cblist_camera.push_back(cb);
}


std::shared_ptr<TisCameraManager::FrameData> TisCameraManager::getNextFrame()
{
    std::unique_lock<std::recursive_mutex> lk(frame->mtx); // this call also locks the thread, with blocking behaviour
    auto ret = frame->con.wait_for(lk,std::chrono::seconds(2)); // with ~0.03ms delay, lock reacquired

    if (ret == std::cv_status::timeout ){
        std::cerr << frame->get_info().topic_ns << ": Wait timeout for new frame arrival..." << std::endl;
        return std::shared_ptr<TisCameraManager::FrameData>();
    }

    return frame;
}