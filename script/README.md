# User Guide
**install-code.sh**:move code to new directory and install
**run-all.sh**:run all sensor driver node except gps
**run-gps.sh**:run gps and its data transform node
**kill.sh**:kill nodes

If you want to run the base node and collect data:
```
cd ~/robot_ws
./run-all.sh
./run-gps.sh
```

If you want to run the navigation system, you also need to put the topological_map.txt into the directory **~/robot_ws/map** and run the node follows:
```
rosrun path_planner app_path_planner
```

