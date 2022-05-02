#include <dv-processing/camera/calibrations/camera_calibration.hpp>
#include <dv-processing/kinematics/transformation.hpp>

#include <dv_ros_capture/capture_node.hpp>

#include <fmt/chrono.h>
#include <fmt/core.h>

#include <chrono>
#include <filesystem>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <utility>

using namespace dv_capture_node;
using namespace dv_ros_msgs;
using namespace std::chrono_literals;

CaptureNode::CaptureNode(ros::NodeHandle &nodeHandle, const dv_ros_node::Params &params) : mParams(params) {
	if (mParams.aedat4FilePath.empty()) {
		mReader = dv_ros_node::Reader(mParams.cameraName);
	}
	else {
		mReader = dv_ros_node::Reader(mParams.aedat4FilePath, mParams.cameraName);
	}

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
		mFramePublisher = nodeHandle.advertise<ImageMessage>("camera/image", 10);
	}

	if (mParams.imu) {
		mImuPublisher = nodeHandle.advertise<ImuMessage>("imu", 1000);
	}

	if (mParams.events) {
		mEventArrayPublisher = nodeHandle.advertise<EventArrayMessage>("events", 1000);
	}

	if (mParams.triggers) {
		mTriggerPublisher = nodeHandle.advertise<TriggerMessage>("triggers", 10);
	}

	if (!mParams.frames && !mParams.events) {
		// Camera info is not published if frames nor events is enabled
		return;
	}

	mCameraInfoPublisher = nodeHandle.advertise<CameraInfoMessage>("camera_info", 10);
	mCameraService       = nodeHandle.advertiseService("set_camera_info", &CaptureNode::setCameraInfo, this);
	mImuInfoService      = nodeHandle.advertiseService("set_imu_info", &CaptureNode::setImuInfo, this);
	mImuBiasesService    = nodeHandle.advertiseService("set_imu_biases", &CaptureNode::setImuBiases, this);

	fs::path calibrationPath = getActiveCalibrationPath();
	if (!mParams.cameraCalibrationFilePath.empty()) {
		ROS_INFO_STREAM("Loading user supplied calibration at path [" << mParams.cameraCalibrationFilePath << "]");
		if (!fs::exists(mParams.cameraCalibrationFilePath)) {
			throw dv::exceptions::InvalidArgument<std::string>(
				"User supplied calibration file does not exist!", mParams.cameraCalibrationFilePath);
		}
		// update/generate active calibration file from used passed camera calibration.
		fs::copy_file(mParams.cameraCalibrationFilePath, calibrationPath, fs::copy_options::overwrite_existing);
	}

	if (fs::exists(calibrationPath)) {
		auto calibSet                = dv::camera::CalibrationSet::LoadFromFile(calibrationPath);
		const std::string cameraName = mReader.getCameraName();
		auto cameraCalibration       = calibSet.getCameraCalibrationByName(cameraName);
		if (const auto &imuCalib = calibSet.getImuCalibrationByName(cameraName); imuCalib.has_value()) {
			mTransformPublisher = nodeHandle.advertise<TransformsMessage>("/tf", 100);
			mImuTimeOffset      = imuCalib->timeOffsetMicros;

			TransformMessage msg;
			msg.header.frame_id = params.imuFrameName;
			msg.child_frame_id  = params.cameraFrameName;

			mImuToCamTransform = dv::kinematics::Transformationf(
				0, Eigen::Matrix<float, 4, 4, Eigen::RowMajor>(imuCalib->transformationToC0.data()));

			mAccBiases.x() = imuCalib->accOffsetAvg.x;
			mAccBiases.y() = imuCalib->accOffsetAvg.y;
			mAccBiases.z() = imuCalib->accOffsetAvg.z;

			mGyroBiases.x() = imuCalib->omegaOffsetAvg.x;
			mGyroBiases.y() = imuCalib->omegaOffsetAvg.y;
			mGyroBiases.z() = imuCalib->omegaOffsetAvg.z;

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
			generateActiveCalibrationFile();
		}
		else {
			throw std::runtime_error("Sensor resolution not available.");
		}
	}

	auto &cameraPtr = mReader.getCameraCapturePtr();
	if (cameraPtr != nullptr) {
		if (cameraPtr->isFrameStreamAvailable()) {
			// DAVIS camera
			davisColorServer
				= std::make_unique<dynamic_reconfigure::Server<dv_ros_capture::DAVISConfig>>(mReaderMutex, nodeHandle);
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
				mReaderMutex, nodeHandle);
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
			= std::make_unique<dynamic_reconfigure::Server<dv_ros_capture::PlaybackConfig>>(mReaderMutex, nodeHandle);
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

	const auto distortion = cameraGeometry.getDistortion();
	mCameraInfoMsg.D.assign(distortion.begin(), distortion.end());
	if (cameraGeometry.getDistortionModel() == dv::camera::DistortionModel::Equidistant) {
		mCameraInfoMsg.distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
	}
	else if (cameraGeometry.getDistortionModel() == dv::camera::DistortionModel::RadTan
			 || cameraGeometry.getDistortionModel() == dv::camera::DistortionModel::None) {
		mCameraInfoMsg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
		if (mCameraInfoMsg.D.size() < 5) {
			mCameraInfoMsg.D.resize(5, 0.0);
		}
	}
	else {
		throw dv::exceptions::InvalidArgument<dv::camera::DistortionModel>(
			"Unknown camera model.", cameraGeometry.getDistortionModel());
	}
	auto cx          = cameraGeometry.getCentralPoint().x;
	auto cy          = cameraGeometry.getCentralPoint().y;
	auto fx          = cameraGeometry.getFocalLength().x;
	auto fy          = cameraGeometry.getFocalLength().y;
	mCameraInfoMsg.K = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
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

bool CaptureNode::setImuBiases(
	dv_ros_capture::SetImuBiases::Request &req, dv_ros_capture::SetImuBiases::Response &rsp) {
	ROS_INFO("Setting IMU biases...");
	if (mParams.unbiasedImuData) {
		ROS_WARN("The IMU data out of this node are unbiased, hence the biases calculation is wrong Not storing "
				 "computed biases.");
		return false;
	}

	mAccBiases.x() = req.accBiases.x;
	mAccBiases.y() = req.accBiases.y;
	mAccBiases.z() = req.accBiases.z;

	mGyroBiases.x() = req.gyroBiases.x;
	mGyroBiases.y() = req.gyroBiases.y;
	mGyroBiases.z() = req.gyroBiases.z;

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

bool CaptureNode::setImuInfo(dv_ros_capture::SetImuInfo::Request &req, dv_ros_capture::SetImuInfo::Response &rsp) {
	mImuTimeOffset = req.imu_info.timeOffsetMicros;
	DV_ROS_MSGS(geometry_msgs::TransformStamped) stampedTransform;
	stampedTransform.transform         = req.imu_info.T_SC;
	stampedTransform.header.frame_id   = mParams.imuFrameName;
	stampedTransform.child_frame_id    = mParams.cameraFrameName;
	mImuToCamTransforms->transforms[0] = stampedTransform;

	Eigen::Quaternion<float> q(static_cast<float>(stampedTransform.transform.rotation.w),
		static_cast<float>(stampedTransform.transform.rotation.x),
		static_cast<float>(stampedTransform.transform.rotation.y),
		static_cast<float>(stampedTransform.transform.rotation.z));
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

fs::path CaptureNode::generateActiveCalibrationFile() const {
	ROS_INFO("Generation active calibration file...");
	std::string cameraName = mReader.getCameraName();
	fs::path calibPath     = fmt::format("{0}/.dv_camera/camera_calibration/{1}", std::getenv("HOME"), cameraName);
	if (!fs::is_directory(calibPath) || !fs::exists(calibPath)) {
		fs::create_directories(calibPath);
	}
	calibPath     = getActiveCalibrationPath();
	auto calibSet = generateCalibrationSet();
	calibSet.writeToFile(calibPath);
	return calibPath;
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

	generateCalibrationSet().writeToFile(calibPath);

	fs::copy_file(calibPath, getActiveCalibrationPath(), fs::copy_options::overwrite_existing);
	return calibPath;
}

dv::camera::CalibrationSet CaptureNode::generateCalibrationSet() const {
	ROS_INFO("Generation calibration set...");
	std::string cameraName = mReader.getCameraName();
	auto calib             = dv::camera::calibrations::CameraCalibration();
	calib.name             = cameraName;
	calib.resolution       = cv::Size(static_cast<int>(mCameraInfoMsg.width), static_cast<int>(mCameraInfoMsg.height));
	for (const auto &d : mCameraInfoMsg.D) {
		calib.distortion.push_back(static_cast<float>(d));
	}
	if (mCameraInfoMsg.distortion_model.c_str() == sensor_msgs::distortion_models::PLUMB_BOB) {
		calib.distortionModel = dv::camera::DistortionModel::RadTan;
	}
	else if (mCameraInfoMsg.distortion_model.c_str() == sensor_msgs::distortion_models::EQUIDISTANT) {
		calib.distortionModel = dv::camera::DistortionModel::Equidistant;
	}
	else {
		throw dv::exceptions::InvalidArgument<dv_ros_msgs::CameraInfoMessage::_distortion_model_type>(
			"Unknown camera model.", mCameraInfoMsg.distortion_model);
	}
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
		const dv::kinematics::Transformationf transform(0,
			Eigen::Vector3f(static_cast<float>(t.x), static_cast<float>(t.y), static_cast<float>(t.z)),
			Eigen::Quaternionf(
				static_cast<float>(r.w), static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z)));
		// Transposing here, because data is saved in RowMajor order instead of ColMajor
		const Eigen::Matrix4f &mat  = transform.getTransform().transpose();
		imuCalib.transformationToC0 = std::vector<float>(mat.data(), mat.data() + mat.rows() * mat.cols());

		imuCalib.accOffsetAvg.x = mAccBiases.x();
		imuCalib.accOffsetAvg.y = mAccBiases.y();
		imuCalib.accOffsetAvg.z = mAccBiases.z();

		imuCalib.omegaOffsetAvg.x = mGyroBiases.x();
		imuCalib.omegaOffsetAvg.y = mGyroBiases.y();
		imuCalib.omegaOffsetAvg.z = mGyroBiases.z();

		calibSet.addImuCalibration(imuCalib);
	}
	return calibSet;
}

void CaptureNode::startCapture() {
	ROS_INFO("Spinning capture node.");
	mSpinThread = true;
	auto times  = mReader.getTimeRange();
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
	if (mCameraInfoThread != nullptr) {
		mCameraInfoThread->join();
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
