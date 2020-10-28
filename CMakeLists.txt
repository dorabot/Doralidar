cmake_minimum_required(VERSION 2.8.3)
project(doralidar)

find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      sensor_msgs
      diagnostic_updater
      dynamic_reconfigure
	  nodelet)

find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(cfg/dlLs.cfg)






catkin_package(
    DEPENDS Boost libusb
    CATKIN_DEPENDS roscpp sensor_msgs diagnostic_updater dynamic_reconfigure
    LIBRARIES dl_ls_udp_lib
    INCLUDE_DIRS include)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


add_library(dl_ls_udp_lib
    src/dl_ls_common.cpp
	src/dl_ls_sensor_frame.cpp
    src/parser_base.cpp)
add_dependencies(dl_ls_udp_lib ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(dl_ls_udp_lib
    ${catkin_LIBRARIES})

add_executable(dl_ls1207de
    src/dl_ls1207de.cpp
        src/dl_ls_common_udp.cpp
    src/dl_ls1207de_parser.cpp)
target_link_libraries(dl_ls1207de dl_ls_udp_lib)

add_library(dl_ls1207de_nodelet
		src/dl_ls1207de_nodelet.cpp
        src/dl_ls_common_udp.cpp
		src/dl_ls1207de_parser.cpp)
target_link_libraries(dl_ls1207de_nodelet ${catkin_LIBRARIES} dl_ls_udp_lib)

#############
## Install ##
#############

install(TARGETS dl_ls1207de_nodelet
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS dl_ls_udp_lib
		DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    TARGETS
        dl_ls1207de
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES include/dl_ls_udp/parser_base.h
              include/dl_ls_udp/dl_ls_common.h
              include/dl_ls_udp/dl_ls_sensor_frame.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
 
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(DIRECTORY plugins/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins)
