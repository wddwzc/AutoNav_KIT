workspace_name="project_ws"

basepath=$(cd `dirname $0`; pwd)
cd $basepath

source ./devel/setup.bash
./kill.sh


# 开启各驱动
sudo chmod 777 /dev/ttyUSB*

cd $basepath
#rosrun xsens_imu_driver xsens_imu_driver &
#sleep 1
#roslaunch dut_mr_drv dur_mr_drv.launch &
#sleep 2
rosrun gnss_driver gnss_driver &
sleep 1
#rosrun Base_control app_Controlservice &
#sleep 2
rosrun gps2odometry gps2odometry &
sleep 1
#roslaunch velodyne_pointcloud VLP16_points.launch &
#sleep 2
# 开启algorithm
#roslaunch laser_slam_algorithm mapping.launch &
#sleep 2
#rosrun Insrob_server app_Insrob_server &
#sleep 2
#rosrun local_map app_local_map &
#sleep 2
#rosrun path_planner app_path_planner &


# 录包
#rostopic hz /velodyne_points &
#rosbag record /velodyne_points &
#rosbag record /velodyne_points /imu/data &
#rosbag record /image_raw &
#rosbag record /image_raw /imu/data &
#rosbag record /gnss/data &
#rosbag record /imu/data /velodyne_points /image_raw /gnss/data &
#rosbag record /imu/data /velodyne_points /image_raw /gnss/data /imu/magneticField /imu/pressure &
#rosbag record /velodyne_points /imu/data /heading/angle /gnss/data &


