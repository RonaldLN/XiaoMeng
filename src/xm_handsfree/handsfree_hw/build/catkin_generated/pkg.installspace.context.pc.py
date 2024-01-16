# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "controller_manager;geometry_msgs;hardware_interface;realtime_tools;roscpp;std_msgs;xm_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lhandsfree_hw".split(';') if "-lhandsfree_hw" != "" else []
PROJECT_NAME = "handsfree_hw"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.1.0"
