#pragma once

#include <dv-processing/camera/calibrations/camera_calibration.hpp>
#include <dv-processing/core/core.hpp>
#include <dv-processing/features/event_combined_lk_tracker.hpp>
#include <dv-processing/features/feature_tracks.hpp>
#include <dv-processing/features/image_feature_lk_tracker.hpp>

// the messaging wants this (?)
#include <dv-processing/io/mono_camera_recording.hpp>

#include <dv_ros_messaging/messaging.hpp>

#include <dv_ros_tracker/TimedKeypointArray.h>

#include <boost/lockfree/spsc_queue.hpp>

#include <ros/ros.h>

#include <atomic>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <thread>
#include <variant>

namespace dv_tracker_node {
using TimedKeypointArrayMessage = DV_ROS_MSGS(dv_ros_tracker::TimedKeypointArray);
using TimedKeypointMessage      = DV_ROS_MSGS(dv_ros_tracker::TimedKeypoint);

using TrackData = std::variant<dv::EventStore, dv_ros_msgs::FrameMap>;

using DataQueue = boost::lockfree::spsc_queue<TrackData, boost::lockfree::capacity<100>>;

namespace dvf = dv::features;

template<typename To, typename From>
[[nodiscard]] inline std::unique_ptr<To> static_unique_ptr_cast(std::unique_ptr<From> &&ptr) {
	return std::unique_ptr<To>(static_cast<To *>(ptr.release()));
}

class TrackerNode {
public:
	TrackerNode(ros::NodeHandle &nodeHandle);
	~TrackerNode();

	void startTracking();

	[[nodiscard]] bool isRunning() const;

private:
	void createTracker();

	enum class OperationMode { EventsOnly, FramesOnly, Combined };

	OperationMode mode = OperationMode::Combined;

	struct TrackerConfig {
		int32_t fastThreshold         = 10;
		bool lookbackRejection        = false;
		int32_t numIntermediateFrames = 5;
		int32_t accumulationFramerate = 50;
		float redetectionThreshold    = 0.75;
		int32_t maxTracks             = 300;
		int32_t numEvents             = 30000;
	};

	dvf::FeatureTracks frameTracks;

	TrackerConfig trackingConfig;
	dvf::ImageFeatureLKTracker::Config lucasKanadeConfig;

	std::thread mKeypointsThread;
	DataQueue mDataQueue;

	ros::Subscriber mEventsArraySubscriber;
	ros::Subscriber mFrameSubscriber;
	ros::Subscriber mFrameInfoSubscriber;

	ros::Publisher mTimedKeypointArrayPublisher;
	ros::Publisher mTracksPreviewPublisher;

	dvf::TrackerBase::UniquePtr tracker = nullptr;

	void stop();

	void eventsArrayCallback(const dv_ros_msgs::EventArrayMessage::ConstPtr &msgPtr);

	void frameCallback(const dv_ros_msgs::ImageMessage::ConstPtr &msgPtr);

	void cameraInfoCallback(const dv_ros_msgs::CameraInfoMessage::ConstPtr &msgPtr);

	std::atomic<bool> mSpinThread;

	dv::camera::calibrations::CameraCalibration mCameraCalibration;
	bool mCameraInitialized = false;
	dvf::FeatureTracks mFrameTracks;

	void assembleTrack();

	[[nodiscard]] inline TimedKeypointArrayMessage toRosTimedKeypointArrayMessage(
		const int64_t timestamp, const dv::cvector<dv::TimedKeyPoint> &keypoints) {
		TimedKeypointArrayMessage msg;

		msg.header.stamp = dv_ros_msgs::toRosTime(timestamp);
		msg.keypoints.reserve(keypoints.size());

		for (const auto &kp : keypoints) {
			auto &k     = msg.keypoints.emplace_back();
			k.x         = kp.pt.x;
			k.y         = kp.pt.y;
			k.size      = kp.size;
			k.angle     = kp.angle;
			k.response  = kp.response;
			k.octave    = kp.octave;
			k.class_id  = kp.class_id;
			k.timestamp = kp.timestamp;
		}

		return msg;
	}
	void pushEventToTracker(const dv::EventStore &events);
	void pushFrameToTracker(const dv::Frame &frame);
	bool runTracking();
	void publishPreview(const cv::Mat &background);
};
} // namespace dv_tracker_node