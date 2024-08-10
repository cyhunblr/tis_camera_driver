# ROS-Driver for TIS USB3 Vision Cameras

Currently tested on Ubuntu 20.04 with ROS Noetic, on an AMD64 system, using a 37AUX462 USB3 Vision camera.

## Camera Driver Installation Guide

The camera driver has two components

- `tiscamera` which is the core driver
- `tiscamera-dutil` which contains many additional useful features, such as tonemapping

### Prequest
1. Install GStreamer
```
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
```
2. Install Python Depencies
```
python3-gi python3-pyqt5
```
2. Uninstall if only if exist tiscamera driver
```
sudo apt remove tiscamera
```
3. For ARM64
```
gstreamer-1.0 libusb-1.0 libglib2.0 libgirepository1.0-dev libudev-dev libtinyxml-dev libzip-dev libnotify-dev
```
##### Build from Debian (.deb)
The files exist in tis_camera_driver/sdk. Just take care that you system AMD64 or ARM64
- `tiscamera_0.12.0` which is the core driver
- `tiscamera-dutils_1.0.0` which contains many additional useful features, such as tonemapping
- Change directory to tis_camera_driver/sdk and you can install as `sudo apt install tiscamera_0.12.0_amd64.deb` and 

**Note:**
- As of Jan 2021, we are using version 0.12.0
- For TX2 or Xavier, if cannot build from source, can try to install the official compiled tiscamera package  `tiscamera_0.12.0_arm64.deb` directly.


However, it is strongly recommanded to build from source, for the main tiscamera driver, especially on Nvidia arm64 platform. https://github.com/TheImagingSource/tiscamera/releases

##### Build from Source

Clone `https://github.com/TheImagingSource/tiscamera` and checkout the release version
```
git clone https://github.com/TheImagingSource/tiscamera.git && cd tiscamera
./scripts/dependency-manager install
git checkout v-tiscamera-0.12.0
mkdir build && cd build
cmake -DBUILD_TOOLS=ON -DBUILD_ARAVIS=OFF ..
make
sudo make install
```
### Installation

Create a workpsace as tis_ws/src and change your directory to tis_ws/src
```
git clone https://github.com/cyhunblr/tis_camera_driver
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### Running 
```
source devel/setup.bash
roslaunch tis_camera_driver tis_camera.launch
```

### Configuration

1. `tis_camera_driver/config` 
- You need to know your camera's serial number and set this parameter in config/configuration.yaml.
- You can view and modify other parameters if necessary.
2. `tis_camera_driver/intrinsics`
- The tis_front_cam.yaml file is the output from the camera_calibration package.
- If you do not set the intrinsic file, camera info will not be published.

### Acknowledgement
([tiscamera_ros](https://github.com/chengguizi/tiscamera_ros/tree/master))

### Issue
1. If tcam lib not found, you have to apply Build from Source. 

#### TODO
- [x] Publish Rectified Image
- [x] Add some other parameter ex. exposure, gain, etc.
- [ ] Clean and prepare the interface
- [ ] Implementation to ROS2
- [ ] Seperate the branch AMD64 and ARM64
