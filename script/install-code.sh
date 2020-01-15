#
#By Wang Zichen
#2020-01
#

#define vars
workspace_name="robot_ws"
basepath=$(cd `dirname $0`; pwd)

# 1) create catkin workspace
echo "---->1. Create catkin space...\n"
#judge if workspace exists?
if [ -d ~/$workspace_name/src ]; then
	echo -n "workspace already exists, overlap ? [y/n]:"
	read res
	if [ $res == "n" ]; then
		exit 0
	else
		rm -fr ~/$workspace_name/
	fi
fi

#create  workspace
mkdir -p ~/$workspace_name/src
cd ~/$workspace_name/src
catkin_init_workspace

cd ~/$workspace_name/
catkin_make

source ~/$workspace_name/devel/setup.bash

#check if success
if echo $ROS_PACKAGE_PATH |grep -a $workspace_name; then
	echo "Successfully create workspace!"
else
	echo "Create workspace failed!"
	exit 1
fi


#2) create pkgs and copy files
echo "---->2. Create pckgs and copy files...\n"

#copy files into the folders
echo ========================$basepath
cd $basepath


cp -rf ./gnss_driver/ ~/$workspace_name/src
cp -rf ./xsens_imu_driver/ ~/$workspace_name/src
cp -rf ./dut_mr_drv/ ~/$workspace_name/src
cp -rf ./gps2odometry/ ~/$workspace_name/src
cp -rf ./Insrob_server/ ~/$workspace_name/src
cp -rf ./local_map/ ~/$workspace_name/src
cp -rf ./path_planner/ ~/$workspace_name/src
cp -rf ./Base_control/ ~/$workspace_name/src
cp -rf ./velodyne-master/ ~/$workspace_name/src
cp -rf ./laser_slam_algorithm/ ~/$workspace_name/src

cp -rf kill.sh ~/$workspace_name/
cp -rf run-all.sh ~/$workspace_name/
cp -rf run-data.sh ~/$workspace_name/
cp -rf run-gps.sh ~/$workspace_name/

mkdir -p ~/$workspace_name/log/
mkdir -p ~/$workspace_name/map/
mkdir -p ~/$workspace_name/data/


#3) catkin make
echo "---->3. Catkin make...\n"
cd ~/$workspace_name/
#catkin_make -DCATKIN_WHITELIST_PACKAGES="velodyne-master"
#catkin_make -DCATKIN_WHITELIST_PACKAGES="xsens_imu_driver"
#catkin_make -DCATKIN_WHITELIST_PACKAGES="laser_slam_algorithm"
#catkin_make -DCATKIN_WHITELIST_PACKAGES="server"

source devel/setup.bash
catkin_make --pkg Base_control
catkin_make --pkg xsens_imu_driver
catkin_make --pkg gnss_driver
catkin_make --pkg dut_mr_drv
catkin_make --pkg gps2odometry
catkin_make --pkg Insrob_server
catkin_make --pkg local_map
catkin_make --pkg path_planner
catkin_make --pkg velodyne-master
catkin_make --pkg laser_slam_algorithm

echo "---->OK!   Install Completely.\n"
