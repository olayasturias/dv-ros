#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/camera/calibrations/camera_calibration.hpp>
#include <dv-processing/kinematics/transformation.hpp>

#include <dv_ros_capture/capture_node.hpp>

#include <fmt/chrono.h>
#include <fmt/core.h>

#include <chrono>
#include <filesystem>
#include <sensor_msgs/image_encodings.h>
#include <unordered_set>
#include <utility>

using namespace dv_capture_node;
using namespace dv_ros_msgs;
using namespace std::chrono_literals;

CaptureNode::CaptureNode(std::shared_ptr<ros::NodeHandle> &nodeHandle, const dv_ros_node::Params &params) :
	mParams(params), mNodeHandle(nodeHandle) {
	mSpinThread = true;
	if (mParams.aedat4FilePath.empty()) {
		mReader = dv_ros_node::Reader(mParams.cameraName);
	}
	else {
		mReader = dv_ros_node::Reader(mParams.aedat4FilePath, mParams.cameraName);
	}
	startupTime = ros::Time::now();
	if (mParams.frames && !mReader.isFrameStreamAvailable()) {
		mParams.frames = false;
		ROS_WARN("Frame stream is not available!");
	}
	if (mParams.events && !mReader.isEventStreamAvailable()) {
		mParams.events = false;
		ROS_WARN("Event stream is not available!");
	}
	if (mParams.imu && !mReader.isImuStreamAvailable()) {
		mParams.imu = false;
		ROS_WARN("Imu data stream is not available!");
	}
	if (mParams.triggers && !mReader.isTriggerStreamAvailable()) {
		mParams.triggers = false;
		ROS_WARN("Trigger data stream is not available!");
	}

	if (mParams.frames) {
		mFramePublisher = mNodeHandle->advertise<ImageMessage>("camera/image", 10);
	}

	if (mParams.imu) {
		mImuPublisher = mNodeHandle->advertise<ImuMessage>("imu", 1000);
	}

	if (mParams.events) {
		mEventArrayPublisher = mNodeHandle->advertise<EventArrayMessage>("events", 1000);
	}

	if (mParams.triggers) {
		mTriggerPublisher = mNodeHandle->advertise<TriggerMessage>("triggers", 10);
	}

	if (!mParams.frames && !mParams.events) {
		// Camera info is not published if frames nor events is enabled
		return;
	}

	mCameraInfoPublisher = mNodeHandle->advertise<CameraInfoMessage>("camera_info", 10);
	mCameraService       = mNodeHandle->advertiseService("set_camera_info", &CaptureNode::setCameraInfo, this);
	mImuInfoService      = mNodeHandle->advertiseService("set_imu_info", &CaptureNode::setImuInfo, this);

	fs::path activeCalibrationPath = getActiveCalibrationPath();
	fs::path calibrationPath;
	if (!mParams.cameraCalibrationFilePath.empty()) {
		ROS_INFO_STREAM("Loading user supplied calibration at path [" << mParams.cameraCalibrationFilePath << "]");
		if (!fs::exists(mParams.cameraCalibrationFilePath)) {
			throw dv::exceptions::InvalidArgument<std::string>(
				"User supplied calibration file does not exist!", mParams.cameraCalibrationFilePath);
		}
		calibrationPath = mParams.cameraCalibrationFilePath;
	}
	else if (fs::exists(activeCalibrationPath)) {
		ROS_INFO_STREAM("Loading auto-detected calibration from [" << activeCalibrationPath << "]");
		calibrationPath = activeCalibrationPath;
	}

	if (!calibrationPath.empty()) {
		auto calibSet                = dv::camera::CalibrationSet::LoadFromFile(calibrationPath);
		const std::string cameraName = mReader.getCameraName();
		auto cameraCalibration       = calibSet.getCameraCalibrationByName(cameraName);
		if (const auto &imuCalib = calibSet.getImuCalibrationByName(cameraName); imuCalib.has_value()) {
			mTransformPublisher = mNodeHandle->advertise<TransformsMessage>("/tf", 100);
			mImuTimeOffset      = imuCalib->timeOffsetMicros;

			TransformMessage msg;
			msg.header.frame_id = params.imuFrameName;
			msg.child_frame_id  = params.cameraFrameName;

			mImuToCamTransform = dv::kinematics::Transformationf(
				0, Eigen::Matrix<float, 4, 4, Eigen::RowMajor>(imuCalib->transformationToC0.data()));

			const auto translation      = mImuToCamTransform.getTranslation<Eigen::Vector3d>();
			msg.transform.translation.x = translation.x();
			msg.transform.translation.y = translation.y();
			msg.transform.translation.z = translation.z();

			const auto rotation      = mImuToCamTransform.getQuaternion();
			msg.transform.rotation.x = rotation.x();
			msg.transform.rotation.y = rotation.y();
			msg.transform.rotation.z = rotation.z();
			msg.transform.rotation.w = rotation.w();

			mImuToCamTransforms = TransformsMessage();
			mImuToCamTransforms->transforms.push_back(msg);
		}
		if (cameraCalibration.has_value()) {
			populateInfoMsg(cameraCalibration->getCameraGeometry());
		}
		else {
			ROS_ERROR_STREAM("Calibration in [" << calibrationPath << "] does not contain calibration for camera ["
												<< cameraName << "]");
			std::vector<std::string> names;
			for (const auto &calib : calibSet.getCameraCalibrations()) {
				names.push_back(calib.second.name);
			}
			const std::string nameString = fmt::format("{}", fmt::join(names, "; "));
			ROS_ERROR_STREAM("The file only contains calibrations for these cameras: [" << nameString << "]");
			throw std::runtime_error("Calibration is not available!");
		}
	}
	else {
		ROS_WARN("No calibration was found, assuming ideal pinhole (no distortion).");
		std::optional<cv::Size> resolution;
		if (mReader.isFrameStreamAvailable()) {
			resolution = mReader.getFrameResolution();
		}
		else if (mReader.isEventStreamAvailable()) {
			resolution = mReader.getEventResolution();
		}
		if (resolution.has_value()) {
			const auto width = static_cast<float>(resolution->width);
			populateInfoMsg(dv::camera::CameraGeometry(
				width, width, width * 0.5f, static_cast<float>(resolution->height) * 0.5f, *resolution));
		}
		else {
			throw std::runtime_error("Sensor resolution not available.");
		}
	}

	auto &cameraPtr = mReader.getCameraCapturePtr();
	if (cameraPtr != nullptr) {
		if (cameraPtr->isFrameStreamAvailable()) {
			// DAVIS camera
			davisColorServer = std::make_unique<dynamic_reconfigure::Server<dv_ros_capture::DAVISConfig>>(
				mReaderMutex, *mNodeHandle);
			dv_ros_capture::DAVISConfig initialSettings    = dv_ros_capture::DAVISConfig::__getDefault__();
			initialSettings.noise_filtering                = mParams.noiseFiltering;
			initialSettings.noise_background_activity_time = static_cast<int>(mParams.noiseBATime);
			davisColorServer->updateConfig(initialSettings);
			davisColorServer->setCallback([this, &cameraPtr](const dv_ros_capture::DAVISConfig &config, uint32_t) {
				cameraPtr->setDavisColorMode(static_cast<dv::io::CameraCapture::DavisColorMode>(config.color_mode));
				cameraPtr->setDavisReadoutMode(
					static_cast<dv::io::CameraCapture::DavisReadoutMode>(config.readout_mode));
				if (config.auto_exposure) {
					cameraPtr->enableDavisAutoExposure();
				}
				else {
					cameraPtr->setDavisExposureDuration(dv::Duration(config.exposure));
				}
				updateNoiseFilter(config.noise_filtering, static_cast<int64_t>(config.noise_background_activity_time));
			});
		}
		else {
			// DVXplorer type camera
			dvxplorerServer = std::make_unique<dynamic_reconfigure::Server<dv_ros_capture::DVXplorerConfig>>(
				mReaderMutex, *mNodeHandle);
			dv_ros_capture::DVXplorerConfig initialSettings = dv_ros_capture::DVXplorerConfig::__getDefault__();
			initialSettings.noise_filtering                 = mParams.noiseFiltering;
			initialSettings.noise_background_activity_time  = static_cast<int>(mParams.noiseBATime);
			dvxplorerServer->updateConfig(initialSettings);
			dvxplorerServer->setCallback([this, &cameraPtr](const dv_ros_capture::DVXplorerConfig &config, uint32_t) {
				cameraPtr->setDVSGlobalHold(config.global_hold);
				cameraPtr->setDVSBiasSensitivity(
					static_cast<dv::io::CameraCapture::BiasSensitivity>(config.bias_sensitivity));
				updateNoiseFilter(config.noise_filtering, static_cast<int64_t>(config.noise_background_activity_time));
			});
		}
	}
	else {
		playbackServer
			= std::make_unique<dynamic_reconfigure::Server<dv_ros_capture::PlaybackConfig>>(mReaderMutex, *mNodeHandle);
		dv_ros_capture::PlaybackConfig initialSettings = dv_ros_capture::PlaybackConfig::__getDefault__();
		initialSettings.noise_filtering                = mParams.noiseFiltering;
		initialSettings.noise_background_activity_time = static_cast<int>(mParams.noiseBATime);
		playbackServer->updateConfig(initialSettings);
		playbackServer->setCallback([this](const dv_ros_capture::PlaybackConfig &config, uint32_t) {
			updateNoiseFilter(config.noise_filtering, static_cast<int64_t>(config.noise_background_activity_time));
		});
	}
}

void CaptureNode::populateInfoMsg(const dv::camera::CameraGeometry &cameraGeometry) {
	mCameraInfoMsg.width  = cameraGeometry.getResolution().width;
	mCameraInfoMsg.height = cameraGeometry.getResolution().height;
	if (cameraGeometry.getDistortionModel() == dv::camera::DistortionModel::Equidistant) {
		mCameraInfoMsg.distortion_model = "fisheye";
	}
	else if (cameraGeometry.getDistortionModel() == dv::camera::DistortionModel::RadTan) {
		mCameraInfoMsg.distortion_model = "plumb_bob";
		if (mCameraInfoMsg.D.size() < 5) {
			mCameraInfoMsg.D.resize(5, 0.0);
		}
	}
	else {
		throw dv::exceptions::InvalidArgument<dv::camera::DistortionModel>(
			"Unknown camera model.", cameraGeometry.getDistortionModel());
	}
	auto cx               = cameraGeometry.getCentralPoint().x;
	auto cy               = cameraGeometry.getCentralPoint().y;
	auto fx               = cameraGeometry.getFocalLength().x;
	auto fy               = cameraGeometry.getFocalLength().y;
	mCameraInfoMsg.K      = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
	const auto distortion = cameraGeometry.getDistortion();
	mCameraInfoMsg.D.assign(distortion.begin(), distortion.end());

	mCameraInfoMsg.R = {1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};
	mCameraInfoMsg.P = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0};
}

bool CaptureNode::setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp) {
	mCameraInfoMsg = req.camera_info;

	try {
		auto calibPath     = saveCalibration();
		rsp.success        = true;
		rsp.status_message = fmt::format("Calibration stored successfully in {0}.", calibPath);
	}
	catch (const std::exception &e) {
		rsp.success        = false;
		rsp.status_message = fmt::format("Error storing camera calibration.");
	}

	return true;
}

bool CaptureNode::synchronizeCamera(
	dv_ros_capture::SynchronizeCamera::Request &req, dv_ros_capture::SynchronizeCamera::Response &rsp) {
	ROS_INFO_STREAM("Received synchronization request from [" << req.masterCameraName << "]");

	// Assume failure case
	rsp.success = false;

	auto &liveCapture = mReader.getCameraCapturePtr();
	if (!liveCapture) {
		ROS_WARN("Received synchronization request on a non-live camera!");
		return true;
	}
	if (liveCapture->isConnected() && !liveCapture->isMasterCamera()) {
		// Update the timestamp offset
		liveCapture->setTimestampOffset(req.timestampOffset);
		ROS_INFO("Camera synchronized: timestamp offset updated.");
		rsp.cameraName = liveCapture->getCameraName();
		rsp.success    = true;
		mSynchronized  = true;
	}
	else {
		ROS_WARN("Received synchronization request on a master camera, please check synchronization cable!");
	}
	return true;
}

bool CaptureNode::setImuInfo(dv_ros_capture::SetImuInfo::Request &req, dv_ros_capture::SetImuInfo::Response &rsp) {
	mImuTimeOffset = req.imu_info.timeOffsetMicros;
	DV_ROS_MSGS(geometry_msgs::TransformStamped) stampedTransform;
	stampedTransform.transform         = req.imu_info.T_SC;
	stampedTransform.header.frame_id   = mParams.imuFrameName;
	stampedTransform.child_frame_id    = mParams.cameraFrameName;
	mImuToCamTransforms->transforms[0] = stampedTransform;

	Eigen::Quaternion<float> q(stampedTransform.transform.rotation.w, stampedTransform.transform.rotation.x,
		stampedTransform.transform.rotation.y, stampedTransform.transform.rotation.z);
	mImuToCamTransform = dv::kinematics::Transformationf(0, Eigen::Vector3f::Zero(), q);

	try {
		auto calibPath     = saveCalibration();
		rsp.success        = true;
		rsp.status_message = fmt::format("Calibration stored successfully in {0}.", calibPath);
	}
	catch (const std::exception &e) {
		rsp.success        = false;
		rsp.status_message = fmt::format("Error storing camera calibration.");
	}

	return true;
}

fs::path CaptureNode::getActiveCalibrationPath() const {
	std::string cameraName = mReader.getCameraName();
	return fmt::format(
		"{0}/.dv_camera/camera_calibration/{1}/active_calibration.json", std::getenv("HOME"), cameraName);
}

fs::path CaptureNode::saveCalibration() const {
	std::string cameraName = mReader.getCameraName();
	fs::path calibPath     = fmt::format("{0}/.dv_camera/camera_calibration/{1}", std::getenv("HOME"), cameraName);
	if (!fs::is_directory(calibPath) || !fs::exists(calibPath)) {
		fs::create_directories(calibPath);
	}

	auto dt   = std::chrono::time_point<std::chrono::system_clock>(dv::Duration(dv::now()));
	auto date = fmt::format("{:%Y_%m_%d_%H_%M_%S}", dt);
	calibPath = fmt::format("{0}/calibration_camera_{1}_{2}.json", calibPath, cameraName, date);

	auto calib       = dv::camera::calibrations::CameraCalibration();
	calib.name       = cameraName;
	calib.resolution = cv::Size(static_cast<int>(mCameraInfoMsg.width), static_cast<int>(mCameraInfoMsg.height));
	for (const auto &d : mCameraInfoMsg.D) {
		calib.distortion.push_back(static_cast<float>(d));
	}
	calib.distortionModel = dv::camera::DistortionModel::RadTan;
	calib.focalLength = cv::Point2f(static_cast<float>(mCameraInfoMsg.K[0]), static_cast<float>(mCameraInfoMsg.K[4]));
	calib.principalPoint
		= cv::Point2f(static_cast<float>(mCameraInfoMsg.K[2]), static_cast<float>(mCameraInfoMsg.K[5]));

	calib.transformationToC0 = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};

	dv::camera::CalibrationSet calibSet;

	calibSet.addCameraCalibration(calib);

	if (mImuToCamTransforms.has_value() && !mImuToCamTransforms->transforms.empty()) {
		auto imuCalib             = dv::camera::calibrations::IMUCalibration();
		imuCalib.timeOffsetMicros = mImuTimeOffset;

		imuCalib.name = cameraName;
		const auto &t = mImuToCamTransforms->transforms.front().transform.translation;
		const auto &r = mImuToCamTransforms->transforms.front().transform.rotation;
		const dv::kinematics::Transformationf transform(
			0, Eigen::Vector3f(t.x, t.y, t.z), Eigen::Quaternionf(r.w, r.x, r.y, r.z));
		// Transposing here, because data is saved in RowMajor order instead of ColMajor
		const Eigen::Matrix4f &mat  = transform.getTransform().transpose();
		imuCalib.transformationToC0 = std::vector<float>(mat.data(), mat.data() + mat.rows() * mat.cols());

		calibSet.addImuCalibration(imuCalib);
	}

	calibSet.writeToFile(calibPath);

	fs::copy_file(calibPath, getActiveCalibrationPath(), fs::copy_options::overwrite_existing);
	return calibPath;
}

void CaptureNode::startCapture() {
	ROS_INFO("Spinning capture node.");
	auto times = mReader.getTimeRange();

	const auto &liveCapture = mReader.getCameraCapturePtr();
	// If the pointer is valid - the reader is handling a live camera
	if (liveCapture) {
		mSynchronized = false;
		mSyncThread   = std::thread(&CaptureNode::synchronizationThread, this);
	}
	else {
		mSynchronized = true;
	}

	if (times.has_value()) {
		mClock = std::thread(&CaptureNode::clock, this, times->first, times->second, mParams.timeIncrement);
	}
	else {
		mClock = std::thread(&CaptureNode::clock, this, -1, -1, mParams.timeIncrement);
	}
	if (mParams.frames) {
		mFrameThread = std::thread(&CaptureNode::framePublisher, this);
	}
	if (mParams.imu) {
		mImuThread = std::thread(&CaptureNode::imuPublisher, this);
	}
	if (mParams.events) {
		mEventsThread = std::thread(&CaptureNode::eventsPublisher, this);
	}
	if (mParams.triggers) {
		mTriggerThread = std::thread(&CaptureNode::triggerPublisher, this);
	}

	if (mParams.events || mParams.frames) {
		mCameraInfoThread = std::make_unique<std::thread>([this] {
			ros::Rate infoRate(25.0);
			while (mSpinThread.load(std::memory_order_relaxed)) {
				const ros::Time currentTime = dv_ros_msgs::toRosTime(mCurrentSeek);
				if (mCameraInfoPublisher.getNumSubscribers() > 0) {
					mCameraInfoMsg.header.stamp = currentTime;
					mCameraInfoPublisher.publish(mCameraInfoMsg);
				}
				if (mImuToCamTransforms.has_value() && !mImuToCamTransforms->transforms.empty()) {
					mImuToCamTransforms->transforms.back().header.stamp = currentTime;
					mTransformPublisher.publish(*mImuToCamTransforms);
				}
				infoRate.sleep();
			}
		});
	}
}

void CaptureNode::updateNoiseFilter(const bool enable, const int64_t backgroundActivityTime) {
	if (enable) {
		// Create the filter and return
		if (mNoiseFilter == nullptr) {
			mNoiseFilter = std::make_unique<dv::noise::BackgroundActivityNoiseFilter<>>(
				mReader.getEventResolution().value(), dv::Duration(backgroundActivityTime));
			return;
		}

		// Noise filter is instantiated, just update the period
		mNoiseFilter->setBackgroundActivityDuration(dv::Duration(backgroundActivityTime));
	}
	else {
		// Destroy the filter
		mNoiseFilter = nullptr;
	}
}

void CaptureNode::clock(int64_t start, int64_t end, int64_t timeIncrement) {
	ROS_INFO("Spinning clock.");

	double frequency = 1.0 / (static_cast<double>(timeIncrement) * 1e-6);

	ros::Rate sleepRate(frequency);
	if (start == -1) {
		start         = std::numeric_limits<int64_t>::max() - 1;
		end           = std::numeric_limits<int64_t>::max();
		timeIncrement = 0;
		ROS_INFO("Reading from camera...");
	}

	while (mSpinThread) {
		if (mSynchronized.load(std::memory_order_relaxed)) {
			if (mParams.frames) {
				mFrameQueue.push(start);
			}
			if (mParams.imu) {
				mImuQueue.push(start);
			}
			if (mParams.events) {
				mEventsQueue.push(start);
			}
			if (mParams.triggers) {
				mTriggerQueue.push(start);
			}
			start += timeIncrement;
		}

		sleepRate.sleep();
		// EOF or reader is disconnected
		if (start >= end || !mReader.isConnected()) {
			mSpinThread = false;
		}
	}
}

void CaptureNode::stop() {
	ROS_INFO("Stopping the capture node.");

	mSpinThread = false;
	mClock.join();
	if (mParams.frames) {
		mFrameThread.join();
	}
	if (mParams.imu) {
		mImuThread.join();
	}
	if (mParams.events) {
		mEventsThread.join();
	}
	if (mParams.triggers) {
		mTriggerThread.join();
	}
	if (mSyncThread.joinable()) {
		mSyncThread.join();
	}
	if (mCameraInfoThread != nullptr) {
		mCameraInfoThread->join();
	}
	if (mDiscoveryThread != nullptr) {
		mDiscoveryThread->join();
	}
}

void CaptureNode::framePublisher() {
	ROS_INFO("Spinning framePublisher");

	std::optional<dv::Frame> frame = std::nullopt;

	while (mSpinThread) {
		mFrameQueue.consume_all([&](const int64_t timestamp) {
			if (!frame.has_value()) {
				std::lock_guard<boost::recursive_mutex> lockGuard(mReaderMutex);
				frame = mReader.getNextFrame();
			}
			while (frame.has_value() && timestamp >= frame->timestamp) {
				if (mFramePublisher.getNumSubscribers() > 0) {
					ImageMessage msg = dv_ros_msgs::frameToRosImageMessage(*frame);
					mFramePublisher.publish(msg);
				}

				mCurrentSeek = frame->timestamp;

				std::lock_guard<boost::recursive_mutex> lockGuard(mReaderMutex);
				frame = mReader.getNextFrame();
			}
		});
		std::this_thread::sleep_for(100us);
	}
}

void CaptureNode::imuPublisher() {
	ROS_INFO("Spinning imuPublisher");

	std::optional<dv::cvector<dv::IMU>> imuData = std::nullopt;

	while (mSpinThread) {
		mImuQueue.consume_all([&](const int64_t timestamp) {
			if (!imuData.has_value()) {
				std::lock_guard<boost::recursive_mutex> lockGuard(mReaderMutex);
				imuData = mReader.getNextImuBatch();
			}
			while (imuData.has_value() && !imuData->empty() && timestamp >= imuData->back().timestamp) {
				if (mImuPublisher.getNumSubscribers() > 0) {
					for (auto &imu : *imuData) {
						imu.timestamp += mImuTimeOffset;
						mImuPublisher.publish(transformImuFrame(dv_ros_msgs::toRosImuMessage(imu)));
					}
				}

				mCurrentSeek = imuData->back().timestamp;

				std::lock_guard<boost::recursive_mutex> lockGuard(mReaderMutex);
				imuData = mReader.getNextImuBatch();
			}
		});
		std::this_thread::sleep_for(100us);
	}
}

void CaptureNode::triggerPublisher() {
	ROS_INFO("Spinning triggerPublisher");

	std::optional<dv::cvector<dv::Trigger>> triggerData = std::nullopt;

	while (mSpinThread) {
		mTriggerQueue.consume_all([&](const int64_t timestamp) {
			if (!triggerData.has_value()) {
				std::lock_guard<boost::recursive_mutex> lockGuard(mReaderMutex);
				triggerData = mReader.getNextTriggerBatch();
			}
			while (triggerData.has_value() && !triggerData->empty() && timestamp >= triggerData->back().timestamp) {
				if (mTriggerPublisher.getNumSubscribers() > 0) {
					for (const auto &trigger : *triggerData) {
						mTriggerPublisher.publish(dv_ros_msgs::toRosTriggerMessage(trigger));
					}
				}

				mCurrentSeek = triggerData->back().timestamp;

				std::lock_guard<boost::recursive_mutex> lockGuard(mReaderMutex);
				triggerData = mReader.getNextTriggerBatch();
			}
		});
		std::this_thread::sleep_for(100us);
	}
}

void CaptureNode::eventsPublisher() {
	ROS_INFO("Spinning eventsPublisher");

	std::optional<dv::EventStore> events = std::nullopt;

	cv::Size resolution = mReader.getEventResolution().value();

	while (mSpinThread) {
		mEventsQueue.consume_all([&](const int64_t timestamp) {
			if (!events.has_value()) {
				std::lock_guard<boost::recursive_mutex> lockGuard(mReaderMutex);
				events = mReader.getNextEventBatch();
			}
			while (events.has_value() && timestamp >= events->getHighestTime()) {
				dv::EventStore store;
				if (mNoiseFilter != nullptr) {
					mNoiseFilter->accept(*events);
					store = mNoiseFilter->generateEvents();
				}
				else {
					store = *events;
				}

				if (mEventArrayPublisher.getNumSubscribers() > 0) {
					auto msg = dv_ros_msgs::toRosEventsMessage(store, resolution);
					mEventArrayPublisher.publish(msg);
				}

				mCurrentSeek = events->getHighestTime();

				std::lock_guard<boost::recursive_mutex> lockGuard(mReaderMutex);
				events = mReader.getNextEventBatch();
			}
		});
		std::this_thread::sleep_for(100us);
	}
}

CaptureNode::~CaptureNode() {
	stop();
}

bool CaptureNode::isRunning() const {
	return mSpinThread.load(std::memory_order_relaxed);
}

void CaptureNode::runDiscovery(const std::string &syncServiceName) {
	const auto &liveCapture = mReader.getCameraCapturePtr();

	if (liveCapture == nullptr) {
		return;
	}

	mDiscoveryPublisher = mNodeHandle->advertise<DiscoveryMessage>("/dvs/discovery", 10);
	mDiscoveryThread    = std::make_unique<std::thread>([this, &liveCapture, &syncServiceName] {
        DiscoveryMessage message;
        message.isMaster         = liveCapture->isMasterCamera();
        message.name             = liveCapture->getCameraName();
        message.startupTime      = startupTime;
        message.publishingEvents = mParams.events;
        message.publishingFrames = mParams.frames;
        message.publishingImu    = mParams.imu;
        message.publishingEvents = mParams.events;
        message.syncServiceTopic = syncServiceName;
        // 5 Hz is enough
        ros::Rate rate(5.0);
        while (mSpinThread) {
            if (mDiscoveryPublisher.getNumSubscribers() > 0) {
                message.header.seq++;
                message.header.stamp = ros::Time::now();
                mDiscoveryPublisher.publish(message);
            }
            rate.sleep();
        }
    });
}

std::vector<std::string> CaptureNode::discoverSyncDevices() const {
	if (mParams.syncDeviceList.empty()) {
		return {};
	}

	ROS_INFO_STREAM(
		"Waiting for devices [" << fmt::format("{}", fmt::join(mParams.syncDeviceList, ", ")) << "] to be online");

	// List info about each sync device
	struct DiscoveryContext {
		std::unordered_set<std::string> discoveredDevices;
		std::vector<std::string> serviceNames;
		std::atomic<bool> complete;
		std::vector<std::string> deviceList;

		void handleMessage(const boost::shared_ptr<DiscoveryMessage> &message) {
			const std::string cameraName(message->name.c_str());
			if (discoveredDevices.contains(cameraName)) {
				return;
			}

			if (std::find(deviceList.begin(), deviceList.end(), cameraName) != deviceList.end()) {
				discoveredDevices.insert(cameraName);
				serviceNames.emplace_back(message->syncServiceTopic.c_str());
				if (serviceNames.size() == deviceList.size()) {
					complete = true;
				}
			}
		}
	};

	DiscoveryContext context;
	context.deviceList = mParams.syncDeviceList;
	context.complete   = false;

	auto subscriber = mNodeHandle->subscribe("/dvs/discovery", 10, &DiscoveryContext::handleMessage, &context);

	while (mSpinThread.load(std::memory_order_relaxed) && !context.complete.load(std::memory_order_relaxed)) {
		std::this_thread::sleep_for(1ms);
	}

	ROS_INFO("All sync devices are online.");

	return context.serviceNames;
}

void CaptureNode::sendSyncCalls(const std::vector<std::string> &serviceNames) const {
	if (serviceNames.empty()) {
		return;
	}

	const auto &liveCapture = mReader.getCameraCapturePtr();
	if (!liveCapture) {
		return;
	}

	dv_ros_capture::SynchronizeCamera srv;
	srv.request.timestampOffset  = liveCapture->getTimestampOffset();
	srv.request.masterCameraName = liveCapture->getCameraName();

	for (const auto &serviceName : serviceNames) {
		ros::ServiceClient client = mNodeHandle->serviceClient<dv_ros_capture::SynchronizeCamera>(serviceName);
		client.waitForExistence();
		if (client.call(srv)) {
			ROS_INFO_STREAM("Successfully synchronized device [" << srv.response.cameraName << "]");
		}
		else {
			ROS_ERROR_STREAM("Failed to synchronize device available on service [" << serviceName << "]");
		}
	}
}

void CaptureNode::synchronizationThread() {
	std::string serviceName;
    const auto &liveCapture = mReader.getCameraCapturePtr();
    if (liveCapture->isMasterCamera()) {
		// Wait for all cameras to show up
		const auto syncServiceList = discoverSyncDevices();
		runDiscovery(serviceName);
		sendSyncCalls(syncServiceList);
		mSynchronized = true;
	}
	else {
		mSyncServerService = std::make_unique<ros::ServiceServer>(mNodeHandle->advertiseService(
			fmt::format("{}/sync", liveCapture->getCameraName()), &CaptureNode::synchronizeCamera, this));

		serviceName = mSyncServerService->getService();
		runDiscovery(serviceName);

		ROS_INFO("Waiting for synchronization service call...");
		while (mSpinThread.load(std::memory_order_relaxed) && !mSynchronized.load(std::memory_order_relaxed)) {
			std::this_thread::sleep_for(1ms);
		}
	}
}
