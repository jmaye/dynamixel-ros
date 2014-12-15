remake_find_package(libdynamixel CONFIG)

remake_include(${LIBDYNAMIXEL_INCLUDE_DIRS})

remake_ros_package_add_library(dynamixel-ros LINK ${LIBDYNAMIXEL_LIBRARIES})
