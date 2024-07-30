#ifndef TISCAMERA_INTERFACE_HPP
#define TISCAMERA_INTERFACE_HPP

#include "tcamcamera.h"

#include <mutex>
#include <condition_variable>
#include <vector>

#include <memory>

#include <cassert>

#include <cstring>

class TisCameraManager : public gsttcam::TcamCamera
{
    public:

        enum TriggerMode{
            NONE = 0,
            TRIGGER_RISING_EDGE,
            TRIGGER_FALLING_EDGE
        };

        class FrameData{
            public:

                struct Info{
                    // Static info
                    std::string topic_ns;
                    std::string camera_sn;
                    int width;
                    int height;
                    int bytes_per_pixel;
                    std::string pixel_format;

                    // Gstreamer Metadata
                    unsigned long meta_api = 0;
                    uint64_t frame_count;
                    uint64_t frames_dropped;
                    uint64_t capture_time_ns;
                    double framerate;
                    int is_damaged; // gboolean is implemented as int
                };

                // FrameData() = default;
                FrameData(std::string topic_ns, std::string camera_sn){
                    _info.topic_ns = topic_ns;
                    _info.camera_sn = camera_sn;
                };
                ~FrameData(){if(_image_data) delete_data();} // prevent memeory leak
                FrameData (const FrameData&) = delete; // disable copy constructor

                
                bool initialised(){
                    std::lock_guard<std::recursive_mutex> lock(mtx);
                    return _initialised;}

                void lock(){
                    mtx.lock();
                }

                bool try_lock(){
                    return mtx.try_lock();
                }

                void unlock(){
                    mtx.unlock();
                }

                // allocating buffer with size
                void allocate(size_t size){
                    std::lock_guard<std::recursive_mutex> lock(mtx);
                    assert(!_image_data && !_initialised);
                    _image_data = new unsigned char[size];
                }

                // write data to an un-initialised by allocated buffer
                void write_data(unsigned char * data, size_t size){
                    std::lock_guard<std::recursive_mutex> lock(mtx);
                    assert(!_initialised);
                    std::memcpy(_image_data, data, size);
                    _initialised = true;
                }

                // de-initialise, but preserve the allocation
                void release(){
                    std::lock_guard<std::recursive_mutex> lock(mtx);
                    // std::cout << "releasing " << _info.capture_time_ns << std::endl;
                    _initialised = false;
                }

                // remove allocation and de-initialise
                void delete_data(){
                    std::lock_guard<std::recursive_mutex> lock(mtx);
                    assert(_image_data);
                    delete [] _image_data; 
                    _image_data = nullptr;
                    _initialised = false;
                }

                const unsigned char * image_data(){
                    std::lock_guard<std::recursive_mutex> lock(mtx);
                    return _image_data;
                }

                inline Info get_info(){
                    std::lock_guard<std::recursive_mutex> lock(mtx);
                    return _info;
                }

                inline void set_info(const Info& info){
                    std::lock_guard<std::recursive_mutex> lock(mtx);
                    _info = info;
                }

                std::condition_variable_any con;
                std::recursive_mutex mtx; // prevent data race between different processes

            private:
                bool _initialised = false;
                // payload
                unsigned char *_image_data = nullptr;
                Info _info; 

        };

        typedef std::function<void(std::shared_ptr<FrameData>)> callbackCamera;

        TisCameraManager(const std::string topic_ns, const std::string serial = "");
        ~TisCameraManager();

        bool set_trigger_mode(TriggerMode value);
        TriggerMode get_trigger_mode();
        bool set_imx_low_latency_mode(bool value);
        bool set_exposure_gain_auto(bool value);
        bool set_exposure_time(int value);
        bool set_exposure_limits(bool auto_upper, int lower, int upper);
        bool set_exposure_auto_reference(int value);
        bool set_gain(int value);
        bool set_gain_limits(int lower, int upper);
        bool set_gamma(float value);
        bool set_tonemapping_mode(bool value);
        bool set_tonemapping_param(float intensity, float global_brightness); // range: -8 to 8, default 1; range: 0 to 1, default 0
        
        bool set_highlight_reduction(bool value);

        void set_capture_format(std::string format, gsttcam::FrameSize size, gsttcam::FrameRate framerate);
        bool start(); // gst playing state
        bool stop();
        void processFrame(); // run in a separate thread
        std::shared_ptr<FrameData> getNextFrame();

        void registerCallback(callbackCamera cb);

        std::string camera_ns;
    
    private:
        bool is_streaming;

        std::shared_ptr<FrameData> frame;

        // below are the mutex protected functions
        GstFlowReturn setFrame(GstAppSink *appsink, gpointer data); // callback from the Gstreamer backend

        // properties
        std::shared_ptr<gsttcam::Property> prop_trigger_mode;
        std::shared_ptr<gsttcam::Property> prop_trigger_polarity;
        std::shared_ptr<gsttcam::Property> prop_trigger_exposure_mode;
        std::shared_ptr<gsttcam::Property> prop_imx_low_latency_mode;

        std::shared_ptr<gsttcam::Property> prop_exposure_mode;
        std::shared_ptr<gsttcam::Property> prop_gain_mode;
        std::shared_ptr<gsttcam::Property> prop_exposure_time;
        std::shared_ptr<gsttcam::Property> prop_gain;
        std::shared_ptr<gsttcam::Property> prop_gamma;

        std::shared_ptr<gsttcam::Property> prop_exposure_auto_reference;

        std::shared_ptr<gsttcam::Property> prop_auto_exposure_upper_limit;

        std::shared_ptr<gsttcam::Property> prop_lower_limit_exposure;
        std::shared_ptr<gsttcam::Property> prop_upper_limit_exposure;
        std::shared_ptr<gsttcam::Property> prop_lower_limit_gain;
        std::shared_ptr<gsttcam::Property> prop_upper_limit_gain;

        std::shared_ptr<gsttcam::Property> prop_tonemapping_mode;
        std::shared_ptr<gsttcam::Property> prop_tonemapping_intensity;
        std::shared_ptr<gsttcam::Property> prop_tonemapping_global_brightness;

        std::shared_ptr<gsttcam::Property> prop_highlight_reduction;

        std::vector<callbackCamera> _cblist_camera;

};

#endif /* TISCAMERA_INTERFACE_HPP */
