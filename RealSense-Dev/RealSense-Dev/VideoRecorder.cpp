#include "VideoRecorder.h"

#include "librealsense2/rs.hpp"
#include <assert.h>
#include <direct.h>

#include <iostream>
#include <filesystem>
#include <vector>
#include <chrono>

#include "Utilities.h"

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
	//this->createContext();
	//this->controlSensorSettings();
}

void VideoRecorder::destroyAll() {

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

			if (verifyOptionSupport(depthSensor, RS2_OPTION_VISUAL_PRESET)) {
				depthSensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
			}

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

void VideoRecorder::startPipeline() {
	this->rsPLProfile = this->rsPipeline.start(this->rsConfig);
}

void VideoRecorder::createContext() {
	if (this->enableDepth) {
		this->rsConfig.enable_stream(RS2_STREAM_DEPTH, 0, 1280, 720, RS2_FORMAT_Z16, this->Depth_FPS);
	}

	if (this->enableRGB) {
		this->rsConfig.enable_stream(RS2_STREAM_COLOR, 0, 1920, 1080, RS2_FORMAT_BGR8, this->RGB_FPS);
	}
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
	if (minutes < 1.0) {
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
			std::cout << dir + "path already exists." << std::endl;
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


void VideoRecorder::recordVideo() {
	try {
		rs2::align alignTo(RS2_STREAM_COLOR);

		int recordedFrameCount = 0;

		// Calculate maximum amount of frames
		int max_frames = this->individualVideoLength * static_cast<float>(this->videoCount) * static_cast<float>(max(this->RGB_FPS, this->Depth_FPS));
		int indVidFrames = this->individualVideoLength * static_cast<float>(max(this->RGB_FPS, this->Depth_FPS));

		cout << this->individualVideoLength << "     " << this->fullSessionLength << endl;
		std::chrono::high_resolution_clock Clock;

		int frameAlignment = this->RGB_FPS / this->Depth_FPS;

		for (int i = 1; i < this->videoCount + 1; i++) {
			std::cout << "Recording video " << i << "of" << this->videoCount << std::endl;
 			
			int frameCount = 0;
			bool framesArrived = false;

			rs2::frameset frameSet;

			std::chrono::duration<float> timeElapsed;
			auto startTime = Clock.now();
			
			timeElapsed = Clock.now() - startTime;


			while (timeElapsed.count() < this->individualVideoLength) {
				framesArrived = this->rsPipeline.try_wait_for_frames(&frameSet, 100);
				if (framesArrived) {
					if (frameCount % frameAlignment == 0){
						if (this->enableDepth && this->enableRGB) {
							frameSet = alignTo.process(frameSet);
							auto colorFrame = frameSet.get_color_frame();
							auto depthFrame = frameSet.get_depth_frame();
						}
					}
					else {
						auto colorFrame = frameSet.get_color_frame();
					}
				
				auto colorMat = frame_to_mat

				}
				


				timeElapsed = Clock.now() - startTime;
			}
		}

	}
	catch (const rs2::error& e) {
	
	}

}