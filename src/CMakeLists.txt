remake_ros_package(
  dynamixel
  DEPENDS roscpp rospy rosbash diagnostic_updater std_msgs sensor_msgs tf
  EXTRA_BUILD_DEPENDS libdynamixel-dev
  EXTRA_RUN_DEPENDS libdynamixel
  DESCRIPTION "Driver for Dynamixel devices."
)
remake_ros_package_config_generate(PACKAGE dynamixel EXTRA_CFLAGS -std=c++0x)
