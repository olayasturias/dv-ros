cmake_minimum_required(VERSION 3.14)
project(dv_ros_runtime_modules VERSION 1.3.0)

find_package(catkin REQUIRED
        COMPONENTS
        pluginlib
        roscpp
        roslib
        dv_ros_msgs
        dv_ros_messaging
)

catkin_package(
        # INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        DEPENDS roscpp roslib dv_ros_msgs dv_ros_messaging
)

INCLUDE(GNUInstallDirs)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/dv /usr/${CMAKE_INSTALL_DATAROOTDIR}/dv
        ${CMAKE_INSTALL_PREFIX}/share/dv /usr/share/dv
        /usr/local/${CMAKE_INSTALL_DATAROOTDIR}/dv /usr/local/share/dv)

# Basic setup
INCLUDE(dv-modules)
DV_MODULE_SETUP()

include_directories(${catkin_INCLUDE_DIRS} ${DV_INCLUDE_DIRS})
# Set full RPATH, modules are libraries for DV
SET(CMAKE_INSTALL_RPATH ${DV_MODULES_DIR})

ADD_LIBRARY(ImageBridge SHARED src/RosImageBridge.cpp)
add_dependencies(ImageBridge ${catkin_EXPORTED_TARGETS})

SET_TARGET_PROPERTIES(ImageBridge
        PROPERTIES
        PREFIX "ros_"
        )
TARGET_LINK_LIBRARIES(ImageBridge PRIVATE ${DV_LIBRARIES} ${catkin_LIBRARIES})

ADD_LIBRARY(ImuBridge SHARED src/RosIMUBridge.cpp)
add_dependencies(ImuBridge ${catkin_EXPORTED_TARGETS})

SET_TARGET_PROPERTIES(ImuBridge
        PROPERTIES
        PREFIX "ros_"
        )

TARGET_LINK_LIBRARIES(ImuBridge PRIVATE ${DV_LIBRARIES} ${catkin_LIBRARIES})

ADD_LIBRARY(EventBridge SHARED src/RosEventBridge.cpp src/RosEventBridge.h)
add_dependencies(EventBridge ${catkin_EXPORTED_TARGETS})

SET_TARGET_PROPERTIES(EventBridge
        PROPERTIES
        PREFIX "ros_"
        )

TARGET_LINK_LIBRARIES(EventBridge PRIVATE ${DV_LIBRARIES} ${catkin_LIBRARIES})

ADD_LIBRARY(TriggerBridge SHARED src/RosTriggerBridge.cpp)
add_dependencies(TriggerBridge ${catkin_EXPORTED_TARGETS})

SET_TARGET_PROPERTIES(TriggerBridge
        PROPERTIES
        PREFIX "ros_"
        )

TARGET_LINK_LIBRARIES(TriggerBridge PRIVATE ${DV_LIBRARIES} ${catkin_LIBRARIES})
