#include <dv_ros_capture/parameters_loader.hpp>

#include <fmt/core.h>

#include <iostream>
#include <string>

using namespace dv_ros_node;

ParametersLoader::ParametersLoader(const ros::NodeHandle &nodeHandle) {
	nodeHandle.getParam("frames", params_.frames);
	nodeHandle.getParam("events", params_.events);
	nodeHandle.getParam("imu", params_.imu);
	nodeHandle.getParam("triggers", params_.triggers);
	nodeHandle.getParam("cameraName", params_.cameraName);
	int timeIncrement = static_cast<int>(params_.timeIncrement);
	nodeHandle.getParam("timeIncrement", timeIncrement);
	params_.timeIncrement = static_cast<int64_t>(timeIncrement);

	nodeHandle.param<std::string>("cameraFrameName", params_.cameraFrameName, "camera");
	nodeHandle.param<std::string>("imuFrameName", params_.imuFrameName, "imu");
	nodeHandle.param<bool>("transformImuToCameraFrame", params_.transformImuToCameraFrame, true);
	nodeHandle.param<bool>("unbiasedImuData", params_.unbiasedImuData, true);

	nodeHandle.param<bool>("noiseFiltering", params_.noiseFiltering, false);
	int value;
	nodeHandle.param<int>("noiseBackgroundActivityTime", value, 2000);
	params_.noiseBATime = static_cast<int64_t>(value);

	std::string tmp;
	nodeHandle.getParam("cameraCalibrationFilePath", tmp);
	params_.cameraCalibrationFilePath = static_cast<std::filesystem::path>(tmp);

	if (nodeHandle.getParam("aedat4FilePath", tmp)) {
		params_.aedat4FilePath = tmp;
		if (!std::filesystem::exists(params_.aedat4FilePath)) {
			throw std::invalid_argument(
				fmt::format("File {0} not found. Please provide a correct path in the configuration file.",
					params_.aedat4FilePath.string()));
		}
	}
}

Params ParametersLoader::getParams() {
	return params_;
}

void ParametersLoader::printConfiguration() {
	ROS_INFO(">>>>>> Capture node parameter settings: <<<<<<");
	// Always print the config, it's easier to understand the behavior
	ROS_INFO_STREAM("Frames enabled: " << (params_.frames ? "yes" : "no"));
	ROS_INFO_STREAM("Events enabled: " << (params_.events ? "yes" : "no"));
	ROS_INFO_STREAM("Imu enabled: " << (params_.imu ? "yes" : "no"));
	ROS_INFO_STREAM("Convert IMU to camera frame: " << (params_.transformImuToCameraFrame ? "yes" : "no"));
	ROS_INFO_STREAM("Triggers enabled: " << (params_.triggers ? "yes" : "no"));
	ROS_INFO_STREAM("aedat4FilePath: " << params_.aedat4FilePath);
	ROS_INFO_STREAM("cameraCalibrationFilePath: " << params_.cameraCalibrationFilePath);
	ROS_INFO_STREAM("noiseFiltering: " << (params_.noiseFiltering ? "yes" : "no"));
	ROS_INFO_STREAM("noiseBackgroundActivityTime: " << params_.noiseBATime);
	ROS_INFO(">>>>>> End of parameters <<<<<<");
}
