# ORB_SLAM3_ROS2
Implementing ORB_SLAM3 in ROS 2 humble with some bonus features.

This repository is meant for running ORB_SLAM3 in ROS 2 humble with a D435i Realsense
camera. If you're looking to run ORB_SLAM3 on a dataset using ROS 2, I suggest
you look at other repositories.

This project is only set up for monocular and imu-monocular modes in orb_slam3
at the moment. I will add support for stereo and RGB-D modes in the next few
weeks (from today, 2024-12-03).

### Building the project

#### Setting up your workspace
The first thing I will do is explain how I set up my ROS 2 workspace for this project.

Make a directory for your ROS 2 workspace, then cd into it:
```sh
mkdir -p ws/src/ && cd ws/src/
```
inside the ```src``` directory, clone this repository and its submodules:
```sh
git clone --recurse-submodules -b main https://github.com/gjcliff/ORB_SLAM3_ROS2.git
```
#### Building ORB_SLAM3
Next, we have to build orbslam3 and its dependencies.

You need to install ORB_SLAM3's dependencies first. Go to [their repo](https://github.com/UZ-SLAMLab/ORB_SLAM3) and follow
their instructions

If you are compiling Pangolin on an Nvidia Jetson, you may need to go into
Pangolin's CMakelists.txt file and remove the -Werror option from ```add_compile_options()```

You can install eigen3 on Ubuntu 22.04 with
```bash
sudo apt install libeigen3-dev
```
and then create a symlink so that ORB_SLAM3 can find it:
```bash
sudo ln -s /usr/include/eigen3/Eigen/ /usr/include/Eigen
```

Next, cd into the ORB_SLAM3 directory and run the ```build.sh``` script. Make sure the
script has execute permissions.
```sh
cd ORB_SLAM3_ROS2/ORB_SLAM3/
./build.sh
```
TODO: Make a dockerfile for the project so that dependencies are set up by default.

#### Building the ROS 2 project
Now you can source ros humble, and the build the project. cd into your ROS 2
workspace and type
```sh
cd -
colcon build
source install/setup.bash
```
### Running the project

There are multiple ways to use this project.

#### ORB_SLAM3

The command to run VIO SLAM using a D435i RealSense camera is:
```sh
ros2 launch orb_slam3_ros2 mapping.launch.xml
```
This will launch the ORB_SLAM3 system in imu-monocular mode by default. You should
see rviz pop up, and then shortly a Pangolin and an opencv window. You should
also see the 3D point cloud from ORB_SLAM3 and an occupancy grid being built
in RVIZ. The map you create will automatically be saved as a filtered
point cloud for the point cloud library (PCL). These files are stored in the
```maps``` directory.

You can record and play back rosbags with arguments to the launch file:
```sh
'record_bag':
    Whether or not to record a rosbag.
    (default: 'false')

'bag_name':
    The name of the bag if record_bag is true. By default, the name of the bag
    will be ORB_SLAM3_YYYY-MM-DD_HH-mm-ss
    (default: 'ORB_SLAM3_YYYY-MM-DD_HH-mm-ss')

'playback_bag':
    The rosbag to play during execution. If set, the realsense2_camera node
    will not launch. Otherwise, nothing will happen.
    (default: 'changeme')
```
Bags are recorded to the ```bags``` directory. The playbag_back argument shouldn't
be a full path to the bag, just the name of it.

#### Localizing
The localization launch file is capable of finding the tf from one occupancy
grid to another. This is useful for localizing maps created by slam_toolbox or
rtabmap in maps create by ORB_SLAM3. To find the tf between the occupancy grids
I use ICP matching through libpointmatcher. The launch file is:
```sh
ros2 launch orb_slam3_ros2 localize.launch.xml
```
You must provide the following argument in order to run localization:
```sh
'reference_map_file':
    The reference map file to localize against
    (default: 'changeme.pcd')
```
This should just be the map file's name, not the full path. Maybe obviously,
you can use maps created by running mapping.launch.py as the reference map file.

### Troubleshooting
1. ORB_SLAM3 keeps resetting the map on its own.
    * Sometimes the map keeps getting lost over and over again over the course of a singular
    run. The best option is just to kill the program and try again.
2. IMU Initialization keeps failing
    * Try and make sure you're moving at the start of the program so that the IMU
    has enough data to initialize. Look for VIBA 1 and VIBA 2 in the terminal output
    (Visual Inertial Bundle Adjustment). However I've noticed that once VIBA 1
    has been completed, you'll likely be able to keep the map even if you stand
    relatively still.
3. Maps look strange or too unlike the real environment
    * I suggest calibrating your camera and IMU. I've provided files, scripts,
    and instructions for doing this [here](./camera_calibration/README.md)
