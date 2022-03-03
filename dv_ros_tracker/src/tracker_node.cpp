#include <dv-processing/data/utilities.hpp>
#include <dv-processing/features/event_feature_lk_tracker.hpp>
#include <dv-processing/visualization/pose_visualizer.hpp>

#include <dv_ros_tracker/tracker_node.h>

#include <iostream>
#include <memory>

using namespace dv_tracker_node;
using namespace std::chrono_literals;

TrackerNode::TrackerNode(ros::NodeHandle &nodeHandle) {
	mFrameInfoSubscriber = nodeHandle.subscribe("camera_info", 10, &TrackerNode::cameraInfoCallback, this);

	mTracksPreviewPublisher      = nodeHandle.advertise<dv_ros_msgs::ImageMessage>("preview/image", 10);
	mTimedKeypointArrayPublisher = nodeHandle.advertise<TimedKeypointArrayMessage>("keypoints", 100);

	lucasKanadeConfig.maskedFeatureDetect
		= nodeHandle.param("maskedFeatureDetect", lucasKanadeConfig.maskedFeatureDetect);
	lucasKanadeConfig.numPyrLayers     = nodeHandle.param("numPyrLayers", lucasKanadeConfig.numPyrLayers);
	int windowSize                     = nodeHandle.param("searchWindowSize", lucasKanadeConfig.searchWindowSize.width);
	lucasKanadeConfig.searchWindowSize = cv::Size(windowSize, windowSize);
	lucasKanadeConfig.terminationEpsilon = nodeHandle.param("terminationEpsilon", lucasKanadeConfig.terminationEpsilon);

	trackingConfig.numIntermediateFrames
		= nodeHandle.param("numIntermediateFrames", trackingConfig.numIntermediateFrames);
	trackingConfig.accumulationFramerate
		= nodeHandle.param("accumulationFramerate", trackingConfig.accumulationFramerate);
	trackingConfig.fastThreshold        = nodeHandle.param("fastThreshold", trackingConfig.fastThreshold);
	trackingConfig.lookbackRejection    = nodeHandle.param("lookbackRejection", trackingConfig.lookbackRejection);
	trackingConfig.redetectionThreshold = nodeHandle.param("redetectionThreshold", trackingConfig.redetectionThreshold);
	trackingConfig.maxTracks            = nodeHandle.param("maxTracks", trackingConfig.maxTracks);
	trackingConfig.numEvents            = nodeHandle.param("numEvents", trackingConfig.numEvents);

	bool useEvents = nodeHandle.param("useEvents", true);
	bool useFrames = nodeHandle.param("useFrames", true);

	if (useEvents && useFrames) {
		mode = OperationMode::Combined;
	}
	else if (useEvents) {
		mode = OperationMode::EventsOnly;
	}
	else if (useFrames) {
		mode = OperationMode::FramesOnly;
	}
	else {
		throw dv::exceptions::RuntimeError(
			"Neither events nor frames are enabled as input, at least one has to be enabled for the tracker!");
	}

	if (mode == OperationMode::FramesOnly || mode == OperationMode::Combined) {
		mFrameSubscriber = nodeHandle.subscribe("image", 10, &TrackerNode::frameCallback, this);
		ROS_INFO("Subscribing to image stream..");

	}
	if (mode == OperationMode::EventsOnly || mode == OperationMode::Combined) {
		mEventsArraySubscriber = nodeHandle.subscribe("events", 10, &TrackerNode::eventsArrayCallback, this);
		ROS_INFO("Subscribing to event stream..");
	}
	frameTracks.setTrackTimeout(10ms);
}

TrackerNode::~TrackerNode() {
	stop();
}

void TrackerNode::eventsArrayCallback(const dv_ros_msgs::EventArrayMessage::ConstPtr &msgPtr) {
	if (msgPtr == nullptr) {
		return;
	}

	auto events = dv_ros_msgs::toEventStore(*msgPtr);

	mDataQueue.push(std::move(events));
}

void TrackerNode::frameCallback(const dv_ros_msgs::ImageMessage::ConstPtr &msgPtr) {
	if (msgPtr == nullptr) {
		return;
	}
	mDataQueue.push(dv_ros_msgs::FrameMap(msgPtr));
}

bool TrackerNode::isRunning() const {
	return mSpinThread.load(std::memory_order_relaxed);
}

void TrackerNode::startTracking() {
	ROS_INFO("Spinning tracking node.");
	mSpinThread      = true;
	mKeypointsThread = std::thread(&TrackerNode::assembleTrack, this);
}

void TrackerNode::createTracker() {
	auto detector = std::make_unique<dvf::ImagePyrFeatureDetector>(
		mCameraCalibration.resolution, cv::FastFeatureDetector::create(trackingConfig.fastThreshold));

	switch (mode) {
		case OperationMode::EventsOnly: {
			auto eventTracker = dvf::EventFeatureLKTracker<dv::PixelAccumulator>::RegularTracker(
				mCameraCalibration.resolution, lucasKanadeConfig, nullptr, std::move(detector),
				std::make_unique<dvf::FeatureCountRedetection>(trackingConfig.redetectionThreshold));
			eventTracker->setMaxTracks(trackingConfig.maxTracks);
			eventTracker->setFramerate(trackingConfig.accumulationFramerate);
			eventTracker->setNumberOfEvents(trackingConfig.numEvents);
			eventTracker->setLookbackRejection(trackingConfig.lookbackRejection);
			tracker = static_unique_ptr_cast<dvf::TrackerBase>(std::move(eventTracker));
			break;
		}
		case OperationMode::FramesOnly: {
			auto frameTracker = dvf::ImageFeatureLKTracker::RegularTracker(mCameraCalibration.resolution,
				lucasKanadeConfig, std::move(detector),
				std::make_unique<dvf::FeatureCountRedetection>(trackingConfig.redetectionThreshold));
			frameTracker->setMaxTracks(trackingConfig.maxTracks);
			frameTracker->setLookbackRejection(trackingConfig.lookbackRejection);
			tracker = static_unique_ptr_cast<dvf::TrackerBase>(std::move(frameTracker));
			break;
		}

		case OperationMode::Combined: {
			auto combinedTracker = dvf::EventCombinedLKTracker<dv::PixelAccumulator>::RegularTracker(
				mCameraCalibration.resolution, lucasKanadeConfig, nullptr, std::move(detector),
				std::make_unique<dvf::FeatureCountRedetection>(trackingConfig.redetectionThreshold));
			combinedTracker->setMaxTracks(trackingConfig.maxTracks);
			combinedTracker->setNumIntermediateFrames(trackingConfig.numIntermediateFrames);
			combinedTracker->setLookbackRejection(trackingConfig.lookbackRejection);
			tracker = static_unique_ptr_cast<dvf::TrackerBase>(std::move(combinedTracker));
			break;
		}
	}
}

void TrackerNode::pushEventToTracker(const dv::EventStore &events) {
	switch (mode) {
		case OperationMode::EventsOnly:
			dynamic_cast<dvf::EventFeatureLKTracker<dv::PixelAccumulator> *>(tracker.get())->accept(events);
			break;
		case OperationMode::Combined:
			dynamic_cast<dvf::EventCombinedLKTracker<dv::PixelAccumulator> *>(tracker.get())->accept(events);
			break;
		default:
			// Noop,
			break;
	}
}

void TrackerNode::pushFrameToTracker(const dv::Frame &frame) {
	switch (mode) {
		case OperationMode::FramesOnly:
			dynamic_cast<dvf::ImageFeatureLKTracker *>(tracker.get())->accept(frame);
			break;
		case OperationMode::Combined:
			dynamic_cast<dvf::EventCombinedLKTracker<dv::PixelAccumulator> *>(tracker.get())->accept(frame);
			break;
		default:
			// Noop
			break;
	}
}

bool TrackerNode::runTracking() {
	if (auto tracks = tracker->runTracking(); tracks != nullptr) {
		frameTracks.accept(tracks);
		mTimedKeypointArrayPublisher.publish(toRosTimedKeypointArrayMessage(tracks->timestamp, tracks->keypoints));
		return true;
	}
	return false;
}

void TrackerNode::publishPreview(const cv::Mat& background) {
	if (mTracksPreviewPublisher.getNumSubscribers() > 0) {
		mTracksPreviewPublisher.publish(
			dv_ros_msgs::toRosImageMessage(frameTracks.visualize(background)));
	}
}

void TrackerNode::assembleTrack() {
	cv::Mat framePreview(mCameraCalibration.resolution.width, mCameraCalibration.resolution.height, CV_8UC3);

	int64_t lastEventsTimestamp = 0;
	std::vector<dv_ros_msgs::FrameMap> frameList;

	createTracker();

	while (mSpinThread) {
		mDataQueue.consume_all([&](const auto &data) {
			if (const dv::EventStore *events = std::get_if<dv::EventStore>(&data); events != nullptr) {
				lastEventsTimestamp = events->getHighestTime();
				pushEventToTracker(*events);
				if (mode == OperationMode::EventsOnly) {
					while (runTracking()) {
						cv::Mat accumulatedImage
							= dynamic_cast<dvf::EventFeatureLKTracker<dv::PixelAccumulator> *>(tracker.get())
								  ->getAccumulatedFrame();
						publishPreview(accumulatedImage);
					}
				}
				else if (mode == OperationMode::Combined) {
					auto it = frameList.begin();
					while (it != frameList.end()) {
						if (it->frame.timestamp < lastEventsTimestamp) {
							pushFrameToTracker(it->frame);
							runTracking();
							it = frameList.erase(it);
						}
						else {
							it++;
						}
					}
				}
			}
			else if (const dv_ros_msgs::FrameMap *map = std::get_if<dv_ros_msgs::FrameMap>(&data); map != nullptr) {
				if (mode == OperationMode::FramesOnly) {
					pushFrameToTracker(map->frame);
					runTracking();
					publishPreview(map->frame.image);
				}
				else {
					frameList.push_back(*map);
					publishPreview(map->frame.image);
				}
			}
			else {
				throw std::runtime_error("Wrong type in queue.");
			}
		});
		std::this_thread::sleep_for(100us);
	}
}

void TrackerNode::cameraInfoCallback(const dv_ros_msgs::CameraInfoMessage::ConstPtr &msgPtr) {
	if (mCameraInitialized) {
		return;
	}

	mCameraCalibration            = dv::camera::calibrations::CameraCalibration();
	mCameraCalibration.resolution = cv::Size(msgPtr->width, msgPtr->height);
	for (const auto &d : msgPtr->D) {
		mCameraCalibration.distortion.push_back(d);
	}
	mCameraCalibration.distortionModel = "plumb_bob";
	mCameraCalibration.focalLength     = cv::Point2f(msgPtr->K[0], msgPtr->K[4]);
	mCameraCalibration.principalPoint  = cv::Point2f(msgPtr->K[2], msgPtr->K[5]);
	mCameraInitialized                 = true;

	startTracking();
}

void TrackerNode::stop() {
	ROS_INFO("Stopping the tracking node.");

	mSpinThread = false;
	mKeypointsThread.join();
}
