#include "VideoRecorder.h"

#include "librealsense2/rs.hpp"
#include <assert.h>
#include <direct.h>

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#include "Utilities.h"
#include <concurrent_queue.h>
#include <opencv-3.4/modules/imgproc/include/opencv2/imgproc.hpp>

using namespace rs2;
using namespace std;


VideoRecorder::VideoRecorder (float individualVideoLength,float fullSessionLength, bool enableRGB, bool enableDepth) {
	this->enableDepth = enableDepth;
	this->enableRGB = enableRGB;

	this->calculateSessionLength(fullSessionLength);
	this->calculateIndividualVidLength(individualVideoLength);
	this->determineOutputVideoCount();
	this->setDirectories();
	this->createDirectories();
	rs2::config rsConfig = this->createContext();
	this->startPipeline(rsConfig);
	this->controlSensorSettings();
}

void VideoRecorder::stopPipeline() {
	this->rsPipeline.stop();
}


bool VideoRecorder::verifyOptionSupport(rs2::sensor rsSensor, rs2_option optionType) {
	if (!rsSensor.supports(optionType))
	{
		std::cerr << "This option is not supported by this sensor" << std::endl;
		return false;
	}

	return true;
}

void VideoRecorder::controlSensorSettings() {
	try {

		/*
		Turn off emitter laser and enable the high accuracy preset.
		*/

		if (this->enableDepth) {
			rs2::depth_sensor depthSensor = this->rsPLProfile.get_device().first<rs2::depth_sensor>();

			//if (verifyOptionSupport(depthSensor, RS2_OPTION_VISUAL_PRESET)) {
			//	depthSensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
			//}

			if (verifyOptionSupport(depthSensor, RS2_OPTION_EMITTER_ENABLED)) {
				depthSensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
			}
		}

		/*
		Turn off the auto-exposure and white balance for RGB sensor.
		*/

		if (this->enableRGB) {
			rs2::color_sensor colorSensor = this->rsPLProfile.get_device().first<rs2::color_sensor>();

			if (verifyOptionSupport(colorSensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
				colorSensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
			}

			if (verifyOptionSupport(colorSensor, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)) {
				colorSensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0);
			}
		}
	}
	catch (const rs2::error& e)
	{
		// Some options can only be set while the camera is streaming,
		// and generally the hardware might fail so it is good practice to catch exceptions from set_option
		std::cerr << "Failed to set option " << ". (" << e.what() << ")" << std::endl;
	}

}

void VideoRecorder::startPipeline(rs2::config config) {
	this->rsPLProfile = this->rsPipeline.start(config);
}

rs2::config VideoRecorder::createContext() { //Todo: Change this to actual video streams!

	rs2:config rsConfig;
	if (this->enableDepth) {
		rsConfig.enable_stream(RS2_STREAM_DEPTH, -1, 1280, 720, RS2_FORMAT_Z16, this->Depth_FPS);
	}

	if (this->enableRGB) {
		rsConfig.enable_stream(RS2_STREAM_COLOR,-1, 1920, 1080, RS2_FORMAT_BGR8, this->RGB_FPS);
	}

	return rsConfig;
}

void VideoRecorder::calculateIndividualVidLength(float min) {
	if (min <= 0.0 ) {
		min = 1.0;
	}
	individualVideoLength = min * 60.0;
}


void VideoRecorder::determineOutputVideoCount() {
	assert(individualVideoLength!=0);
	int count = fullSessionLength / individualVideoLength;
		
	if (count < 1) {
		count = 1;
	}

	videoCount = count;
}

void VideoRecorder::calculateSessionLength(float minutes) {
	if (minutes <= 0.0) {
		minutes = 1.0;
	}
	fullSessionLength = minutes * 60.0;
}

void VideoRecorder::createDirectories() {
	vector<string> directories = { this->parentDir,
								   this->baseDir,
								   this->colorDir,
								   this->depthDir};

	int failure = 0;

	for(string dir : directories)
	{

		if (isPathExist(dir)) {
			continue;
		}

		else {
			failure = _mkdir(dir.c_str());

			if (failure == 1) {
				std::cerr << "Failed to create directory for:" + dir << endl;
				return;
			}
		}
	}
}


void VideoRecorder::setDirectories() {
	std::time_t now = std::time(0);
	tm timeinfo[80];
	char time_buffer[80];
	
	localtime_s(timeinfo, &now);

	strftime(time_buffer, 80, "%Y_%m_%d-%H_%M", timeinfo);

	this->baseDir = this->parentDir +  "output_" + string(time_buffer) + "/";
	this->colorDir = this->baseDir + "color/";
	this->depthDir = this->baseDir + "depth/";
}


void VideoRecorder::verifySetUp() {
	rs2::align alignTo(RS2_STREAM_COLOR);
	rs2::colorizer colorMap;

	// Visualization of images.
	const std::string depthWindowName = "Depth Image";
	const std::string colorWindowName = "Color Image";
	cv::namedWindow(depthWindowName, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(colorWindowName, cv::WINDOW_AUTOSIZE);

	rs2::frameset frameSet;

	cv::Mat depthImage;
	cv::Mat colorImage;

	rs2::frame colorFrame;
	rs2::frame depthFrame;
	std::chrono::high_resolution_clock Clock;

	bool framesArrived = true;
	std::chrono::duration<float> timeElapsed;
	auto startTime = Clock.now();
	timeElapsed = Clock.now() - startTime;

	int frame_count = 0;

	while (true) {

		timeElapsed = Clock.now() - startTime;

		if (timeElapsed.count() >= 5){
			int avgfps = frame_count / timeElapsed.count();
			cout << "Average FPS:" << avgfps << endl;
			startTime = Clock.now();
			frame_count = 0;
		}

		frameSet = this->rsPipeline.wait_for_frames(1000);

		if (framesArrived) {


			frameSet = alignTo.process(frameSet);
			colorFrame = frameSet.get_color_frame();
			depthFrame = frameSet.get_depth_frame();


			colorImage = frame_to_mat(colorFrame);
			depthImage = frame_to_mat(depthFrame);

			cv::convertScaleAbs(depthImage, depthImage, 0.03);
			cv::applyColorMap(depthImage, depthImage, cv::ColormapTypes::COLORMAP_JET);
			if (colorImage.size().width != 0 && colorImage.size().height != 0 && depthImage.size().width != 0 && depthImage.size().height != 0) {
				cv::imshow(colorWindowName, colorImage);
				cv::imshow(depthWindowName, depthImage);
			}
			auto key = (char)27;

			if (cv::waitKey(1) == key) {
				cv::destroyAllWindows();
				break;
			}
		}

		frame_count += 1;
	}
	
	depthImage.release();
	colorImage.release();

	colorFrame.~frame();
	depthFrame.~frame();
}


void VideoRecorder::recordVideo() {
	
	//int recordingFps = max(this->RGB_FPS, this->Depth_FPS);

	rs2::frame_queue depthFramesQueue(10);
	rs2::frame_queue colorFramesQueue(10);

	std::this_thread::sleep_for(std::chrono::seconds(10));

	std::thread depthSavingThread(writeFrames, depthFramesQueue, "depth", this->depthDir, this->baseDir, this->videoCount, this->individualVideoLength, 6);
	std::thread colorSavingThread(writeFrames, colorFramesQueue, "color", this->colorDir, this->baseDir, this->videoCount, this->individualVideoLength, this->RGB_FPS);

	try {
		// Depth colorization and alignment.
		rs2::align alignTo(RS2_STREAM_COLOR);
		rs2::colorizer colorMap;

		// Information tracking
		int recordedFrameCount = 0;
		int maxFrames =  static_cast<int>(this->fullSessionLength * max(this->RGB_FPS, this->Depth_FPS));

		// Calculate maximum amount of frames
		int max_frames = static_cast<int>(this->individualVideoLength * static_cast<float>(this->videoCount) * max(this->RGB_FPS, this->Depth_FPS));
		
		// Create clock
		std::chrono::high_resolution_clock Clock;

		// Check for when frames must be aligned.
		int frameAlignment = static_cast<int>(this->RGB_FPS / this->Depth_FPS);

		// Object for frames
		rs2::frameset frameSet;

		rs2::frame colorFrame;
		rs2::frame depthFrame;

		int frameCount = 0;
		bool framesArrived = false;
			

		// Keep track of time in video.
		std::chrono::duration<float> timeElapsed;
		auto startTime = Clock.now();
		timeElapsed = Clock.now() - startTime;
		// Record a video
		while (timeElapsed.count() < this->fullSessionLength) {

			timeElapsed = Clock.now() - startTime;

			framesArrived = this->rsPipeline.try_wait_for_frames(&frameSet, 300);

			if (framesArrived) {
				if (frameCount % 5 == 0) {
					frameSet = alignTo.process(frameSet);
					colorFrame = frameSet.get_color_frame();
					depthFrame = frameSet.get_depth_frame(); // .apply_filter(colorMap);

					depthFramesQueue.enqueue(depthFrame);
					colorFramesQueue.enqueue(colorFrame);
				}
				else {
					colorFrame = frameSet.get_color_frame();
					colorFramesQueue.enqueue(colorFrame);
				}
			}
			else 
			{
				throw rs2::error("Frames did not arrive on time.");
			}
				
			// Updating time loop and frames
			frameCount += 1;

			// Debug printing
			if (frameCount % 150 == 0) {
				std::cout << frameCount << " at " << timeElapsed.count() << " seconds." << endl;
			}
		}

	
		recordedFrameCount += frameCount;

		colorSavingThread.join();
		depthSavingThread.join();

		std::cout << "Number of frames captured:" << recordedFrameCount << endl;
		std::cout << "Number of max possible frames:" << maxFrames << endl;

	}
	catch (const rs2::error& e) {
		std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
		colorSavingThread.join();
		depthSavingThread.join();
		this->stopPipeline();
		return;
	}

	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		colorSavingThread.join();
		depthSavingThread.join();
		this->stopPipeline();
		return;
	}

	return;
}