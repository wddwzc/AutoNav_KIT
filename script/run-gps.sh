workspace_name="robot_ws"

basepath=$(cd `dirname $0`; pwd)
cd $basepath

# 开启各驱动
sudo chmod 777 /dev/ttyUSB*

roslaunch gps2odometry gps.launch
