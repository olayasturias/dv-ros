#pragma once

#include <dv-processing/camera/calibrations/camera_calibration.hpp>
#include <dv-processing/core/core.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>
#include <dv-processing/noise/background_activity_noise_filter.hpp>

#include <dv_ros_messaging/messaging.hpp>

#include <dv_ros_capture/parameters_loader.hpp>
#include <dv_ros_capture/reader.hpp>
#include <dv_ros_capture/DAVISConfig.h>
#include <dv_ros_capture/DVXplorerConfig.h>
#include <dv_ros_capture/PlaybackConfig.h>
#include <dv_ros_capture/SetImuInfoService.h>

#include <boost/lockfree/spsc_queue.hpp>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <thread>

namespace dv_capture_node {
using TimestampQueue = boost::lockfree::spsc_queue<int64_t, boost::lockfree::capacity<1000>>;

using TransformMessage = DV_ROS_MSGS(geometry_msgs::TransformStamped);
using TransformsMessage = DV_ROS_MSGS(tf2_msgs::TFMessage);

namespace fs = std::filesystem;

/**
 * Initialize ros publishers to stream data from a DV camera or a aedat4 file.
 */
class CaptureNode {
public:
	/**
	 * Initialize the publisher according to the params.
	 * @param nodeHandle ros::NodeHandle.
	 * @param params dv_ros_node::Params read from a configuration file specified in the launch file.
	 */
	CaptureNode(ros::NodeHandle &nodeHandle, const dv_ros_node::Params& params);

	/**
	 * Stop the running threads.
	 */
	~CaptureNode();

	/**
	 * Start the threads for reading the data.
	 */
	void startCapture();

	/**
	 * Stop the running threads.
	 */
	void stop();

	/**
	 * Check whether read threads are still running
	 * @return True if capture threads are still running, false otherwise.
	 */
	[[nodiscard]] bool isRunning() const;

private:
	// Publishers declaration.
	ros::Publisher mFramePublisher;
	ros::Publisher mCameraInfoPublisher;
	ros::Publisher mImuPublisher;
	ros::Publisher mEventArrayPublisher;
	ros::Publisher mTriggerPublisher;
	ros::Publisher mTransformPublisher;
	ros::ServiceServer mCameraService;
	ros::ServiceServer mImuInfoService;

	// Declare the Reader to read from the device or from a recording.
	dv_ros_node::Reader mReader;

	// Paramters read from the configuration file. Set to true the type of data that needs to be streamed.
	dv_ros_node::Params mParams;

	// Camera info message buffer
	dv_ros_msgs::CameraInfoMessage mCameraInfoMsg;
	std::unique_ptr<std::thread> mCameraInfoThread = nullptr;

	// threads related
	std::thread mFrameThread;
	TimestampQueue mFrameQueue;
	std::thread mImuThread;
	TimestampQueue mImuQueue;
	std::thread mEventsThread;
	TimestampQueue mEventsQueue;
	std::thread mTriggerThread;
	TimestampQueue mTriggerQueue;
	std::atomic<bool> mSpinThread = true;
	std::thread mClock;
	boost::recursive_mutex mReaderMutex;

	bool mEnableNoiseFilter = false;
	std::unique_ptr<dv::noise::BackgroundActivityNoiseFilter<>> mNoiseFilter = nullptr;
	void updateNoiseFilter(const bool enable, const int64_t backgroundActivityTime);

	int64_t mImuTimeOffset=0;
	std::atomic<int64_t> mCurrentSeek;
	std::optional<TransformsMessage> mImuToCamTransforms = std::nullopt;

	std::unique_ptr<dynamic_reconfigure::Server<dv_ros_capture::DAVISConfig>> davisColorServer    = nullptr;
	std::unique_ptr<dynamic_reconfigure::Server<dv_ros_capture::DVXplorerConfig>> dvxplorerServer = nullptr;
	std::unique_ptr<dynamic_reconfigure::Server<dv_ros_capture::PlaybackConfig>> playbackServer = nullptr;

	/**
	 * Publish the images and camera info if mFrameBool is True
	 */
	void framePublisher();
	bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp);

	/**
	 * Publish the imu message if mImuBool is True
	 */
	void imuPublisher();
	bool setImuInfo(dv_ros_capture::SetImuInfo::Request &req, dv_ros_capture::SetImuInfo::Response &rsp);

	/**
	 * Publish the events if mEventsBool is True
	 */
	void eventsPublisher();

	/**
	 * Publish the triggers if mTriggerBool is True
	 */
	void triggerPublisher();

	/**
	 * Start a clock thread that gives time synchronization to all the other threads.
	 * @param start Start time of a recording file. (-1 if capturing from camera)
	 * @param end End time of a recording file. (-1 if capturing from camera)
	 * @param timeIncrement Increment of the timestamp at each iteration of the thread. The thread sleeps for
	 * timeIncrement micros.
	 */
	void clock(int64_t start, int64_t end, int64_t timeIncrement);

	/**
	 * Create a camera info message.
	 * @param cameraCalib camera parameters for the camera info message.
	 */
	void populateInfoMsg(const dv::camera::CameraGeometry &cameraGeometry);

	[[nodiscard]] fs::path saveCalibration() const;

	[[nodiscard]] fs::path getActiveCalibrationPath() const;
};

} // namespace dv_capture_node