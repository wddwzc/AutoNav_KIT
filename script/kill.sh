
killall record

#kill algorithm
killall -9 transform
killall -9 mapping
killall -9 odom
killall -9 register
killall -9 relocalization


#kill velodyne drivers
kill $(ps aux|grep "velodyne"|awk '{print $2}')


#这几个节点需要友好退出（使while rosok失效），因为结束时需要相关配置
#killall -9 flea3_async_trigger
rosnode kill flea3_async_trigger1
rosnode kill flea3_async_trigger2
#killall -9 xsens_imu_driver
rosnode kill xsens_imu_driver
#killall -9 gnss_driver
rosnode kill gnss_driver

#killall -9 server





