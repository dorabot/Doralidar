# Doralidar ROS Driver

ROS Driver package for Doralidar series

- DL-QMT20-H2
- DL-QOA06PNP-H3
- DL-QMS40PNP-H2

## Before install

Environment

1. **`Ubuntu 18.04/20.04`**
2. **`ROS Melodic`**

## How to Build

```shell
which_shell=$(echo $SHELL|sed 's;^.*/;;g')

# 1. Setup environment
source /opt/ros/melodic/setup.${which_shell}

# 2. Install
catkin_make install
```

## How to Run

```shell
# 1. Run with node
roslaunch doralidar dl_ls1207de_with_1_lidar.launch

# 2. Run with nodelet
roslaunch doralidar dl_ls1207de_nodelet_with_1_lidar.launch
```

## Note

Please modify `range_max` param in launch file according to the type of Doralidar you are running. (20/40)

### How to disable debug mode

In `cfg/dlLs.cfg`, set the default value of `debug_mode` to `False`

### How to disable check frame mode

In `dl_ls1207de.launch` or `dl_ls1207de_nodelet.launch`, set the `checkframe = false` in xml 
