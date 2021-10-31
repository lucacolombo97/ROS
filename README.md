# Robotics Projects

### Homework 1

**Description of the src files:**
- pub_sub: convert from lla to ENU and publish TF and Odom for both car and obstacle
- custom_msg: publish a custom message with fields: distance between car and obstacle and status flag
- compute_distance: the service that returns the distance between the two vehicles 

**The parameters for dynamic reconfiguration are:**
- unsafe_param: threshold for classifying safe/unsafe distance (default: 5m)
- crash_param: threshold for classifying crash distance (default: 1m)
If: 
- crash_param < dist < unsafe_param -> "Unsafe"
- dist > unsafe_param -> "Safe"
- dist < crash_param -> "Crash"
If the parameters are negative or crash_param > unsafe_param the modification is ignored
- lat_param, long_param, h_param: starting point

**Structure of the tf tree:**
![Tf_Tree](https://user-images.githubusercontent.com/48442855/139583001-a40f63ef-d715-4dac-89f5-b673ee48c8bf.png)

**Structure of custom_msg:**
- dist: distance between car and obstacle
- flag: status of the car

**How start the nodes:**
Inside the launch folder, type command "roslaunch project1.launch"

**Other details:**
- In Pub_sub to choose between car and obstacle is passed a number as input, 0 for the car and 1 for the obstacle
- At the beginning, when obs is not yet started distance is set to -1 so the custom msg is not published
- If there is a nan, custom_msg publish a message with nan for distance and "GPS signal lost" for flag
#
### Homework 2.
**Description:**
Firt we have converted the 3D PointCloud to laserscan to use with GMapping and then we have created 2 maps:
1. The first one using the Optitrack odometry
2. The second one with the visual odometry  

Then we have improved the visual odometry with IMUs data for the localization and navigation of the robot

**Description of the files:**
- amcl.launch: launch file used to localize the robot in the given 2d map
- amcl_imu.launch: launch file used to localize the robot in the given 2d map improved through IMUs data
- gmapping.launch: launch file used to create the map using gmapping with visual odometry
- gmapping_optitrack: launch file used to create the map using gmapping with optitrack odometry
- gmapping_imu.launch: launch file used to create the map using gmapping with visual odometry improved through IMUs data

Name of bag used for create the map: 2020-05-14-16-14-37-traj2-os1-t265-pix  
Name of bag used for localization: 2020-05-14-16-09-36-traj1-os1-t265-pix

**Structure of the AMCL tf tree:**
![Tf_Tree_AMCL](https://user-images.githubusercontent.com/48442855/139583133-9e1d0614-b529-4d72-9356-2a71473e481d.png)

**Structure of the GMapping tf tree:**
![Tf_Tree_GMapping](https://user-images.githubusercontent.com/48442855/139583148-6803c5fd-da0e-4f05-b86f-1bff85e7cbdc.png)

**How start the nodes:**
Inside the launch folder type the command "roslaunch gmapping.launch".  
To start the others launch files, use the same command with the different name of the launch file.

**Other details:**
- In the map there are some errors although different parameters are been tested.
- Pointcloud to laserscan has been used to convert a 3d pointcloud into a 2d laserscan.
- To fuse visual odometry with imus data we have used the robot pose ekf.
