#include <dv-processing/io/mono_camera_recording.hpp>

#include <dv_ros_messaging/messaging.hpp>

#include <CLI/CLI.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <filesystem>

void progressBar(const float progress) {
	int barWidth = 70;

	std::cout << "[";
	int pos = static_cast<int>(static_cast<float>(barWidth) * progress);
	for (int i = 0; i < barWidth; ++i) {
		if (i < pos)
			std::cout << "=";
		else if (i == pos)
			std::cout << ">";
		else
			std::cout << " ";
	}
	std::cout << "] " << int(progress * 100.0) << " %\r";
	std::cout.flush();
}

int main(int argc, char **argv) {
	namespace fs = std::filesystem;

	CLI::App app{"Aedat4 to rosbag converter. Supports single camera recordings with events, images, imu, and trigger "
				 "data streams."};

	std::string input;
	std::string output;
	std::string topicNamespace = "/recording";
	bool overwrite             = false;
	bool verbose               = false;
	app.add_option("-i,--input", input, "Input aedat4 file")->required()->check(CLI::ExistingFile);
	app.add_option("-o,--output", output,
		"Output rosbag file path, if not provided, will create file with same path as input, but with .bag extension");
	app.add_flag("-n,--namespace", topicNamespace, "Topic namespace");
	app.add_flag("-f,--force-overwrite", overwrite);
	app.add_flag("-v,--verbose", verbose);

	CLI11_PARSE(app, argc, argv);

	dv::io::MonoCameraRecording reader(input);

	if (output.empty()) {
		output = fs::path(input).replace_extension(".bag").string();
	}

	if (fs::exists(output) && !overwrite) {
		std::cout << "Output file already exists. Do you want to overwrite? [y/n]" << std::endl;
		char in;
		std::cin >> in;
		if (in != 'y') {
			std::cout << "Exiting due to user input" << std::endl;
			return EXIT_SUCCESS;
		}
	}

	rosbag::Bag bag(output, rosbag::bagmode::Write);

	dv::io::DataReadHandler handler;
	if (reader.isFrameStreamAvailable()) {
	}

	handler.mFrameHandler = [&bag, &topicNamespace](const dv::Frame &frame) {
		auto msg = dv_ros_msgs::frameToRosImageMessage(frame);
		bag.write(topicNamespace + "/image", msg.header.stamp, msg);
	};

	if (reader.isEventStreamAvailable()) {
		cv::Size resolution = reader.getEventResolution().value();

		handler.mEventHandler = [&bag, &topicNamespace, &resolution](const dv::EventStore &events) {
			if (events.isEmpty()) {
				return;
			}

			auto msg = dv_ros_msgs::toRosEventsMessage(events, resolution);
			bag.write(topicNamespace + "/events", dv_ros_msgs::toRosTime(events.getLowestTime()), msg);
		};
	}

	handler.mImuHandler = [&bag, &topicNamespace](const dv::cvector<dv::IMU> &imuBatch) {
		for (const auto &imu : imuBatch) {
			auto msg = dv_ros_msgs::toRosImuMessage(imu);
			bag.write(topicNamespace + "/imu", msg.header.stamp, msg);
		}
	};

	handler.mTriggersHandler = [&bag, &topicNamespace](const dv::cvector<dv::Trigger> &triggerBatch) {
		for (const auto &trigger : triggerBatch) {
			auto msg = dv_ros_msgs::toRosTriggerMessage(trigger);
			bag.write(topicNamespace + "/triggers", msg.timestamp, msg);
		}
	};

	const auto startEnd = reader.getTimeRange();
	const auto duration = static_cast<float>(startEnd.second - startEnd.first);
	while (reader.handleNext(handler)) {
		const float progress = static_cast<float>((handler.seek - startEnd.first)) / duration;
		if (verbose) {
			progressBar(progress);
		}
	}

	bag.close();

	return EXIT_SUCCESS;
}
