cmake_minimum_required(VERSION 3.14)
project(dv_ros_aedat4 VERSION 1.2.0)

# Git submodule automatic update support.
find_package(Git QUIET)

if(Git_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../.git")
  # Update submodules as needed
  option(GIT_SUBMODULE "Check-out submodules during build" ON)

  if(GIT_SUBMODULE)
    message(STATUS "Git submodule update")

    execute_process(
      COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../
      RESULT_VARIABLE GIT_SUBMOD_RESULT)
    if(NOT GIT_SUBMOD_RESULT EQUAL "0")
      message(
        FATAL_ERROR
          "${GIT_EXECUTABLE} submodule update --init failed with ${GIT_SUBMOD_RESULT}, please check-out submodules manually."
      )
    endif()
  endif()
endif()

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             rosbag
             std_msgs
             sensor_msgs
             image_transport
             visualization_msgs
             dv_ros_msgs
             dv_ros_messaging)

find_package(dv-processing 1.4.0 REQUIRED)

add_subdirectory(thirdparty/CLI11)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  sensor_msgs
  dv_ros_msgs
  dv_ros_messaging
  rosbag)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(convert_aedat4 src/convert_aedat4.cpp)

add_dependencies(convert_aedat4 ${catkin_EXPORTED_TARGETS}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(convert_aedat4 ${catkin_LIBRARIES} dv::processing
                      CLI11::CLI11)
