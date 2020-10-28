# Doralidar ROS Driver
  ROS Driver package for Doralidar series:
* DL-QMT20-H2
* DL-QOA06PNP-H3
* DL-QMS40PNP-H2

## BUILD AND RUN
### 1: build:
	catkin_make
### 2: install:
	catkin_make install
### 3.1: run with node:
	roslaunch doralidar dl_ls1207de_with_1_lidar.launch
### 3.2: run with nodelet:
	roslaunch doralidar dl_ls1207de_nodelet_with_1_lidar.launch

### Note:
Please modify "range_max" param in launch file according to the type of Doralidar you are running. (20/40)

### HOW TO DISABLE DEBUG MODE
In cfg/dlLs.cfg, set the default value of debug_mode to False

#### HOW TO DISABLE CHECK FRAME MODE
In dl_ls1207de.launch or dl_ls1207de_nodelet.launch, set the checkframe = false in xml 