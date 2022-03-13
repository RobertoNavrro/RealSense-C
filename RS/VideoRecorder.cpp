#include "VideoRecorder.h"

#include "librealsense2/rs.hpp"
#include <assert.h>
#include <direct.h>

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <fstream>

#include "Utilities.h"
#include "ROIHolder.h"
#include "VideoController.h"
//#include <WinUser.h>
//#include <opencv-3.4/modules/imgproc/include/opencv2/imgproc.hpp>

using namespace rs2;
using namespace std;

void VideoRecorder::setDepthROIDefault(int width, int height) {
	int origin_x, origin_y;
	int roi_width, roi_height;

	roi_width = int(0.5 * width);
	roi_height = int(0.5 * height);

	this->depthROI.width = roi_width;
	this->depthROI.height = roi_height;

	this->depthROI.setOriginPoint(int(0.25 * width), int(0.25 * height));
	this->depthROI.setEndPoint(this->depthROI.origin.x + roi_width, this->depthROI.origin.y + roi_height);

	this->setNewDepthROI();
}


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
	this->createVideoController();
	this->writeIntrinsics();
	this->writeExtrinsics();
	this->writeDepthDeviceInformation();
	this->setDepthROIDefault(1080, 720);
}

void VideoRecorder::createVideoController() {
	rs2::sensor depthSensor = this->rsPLProfile.get_device().first<rs2::depth_sensor>();
	rs2::sensor colorSensor = this->rsPLProfile.get_device().first<rs2::color_sensor>();
	this->videoController = VideoController(colorSensor, depthSensor, "Preview");
}

void VideoRecorder::stopPipeline() {
	this->rsPipeline.stop();
}

void VideoRecorder::writeExtrinsics() {
	auto videoStream = this->rsPipeline.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	auto colorStream = this->rsPipeline.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	rs2_extrinsics extrinsicsDepthtoColor = videoStream.get_extrinsics_to(colorStream);
	rs2_extrinsics extrinsicsColortoDepth = colorStream.get_extrinsics_to(videoStream);
	this->saveExtrinsics(extrinsicsDepthtoColor, "extrinsics_depth_to_color.json");
	this->saveExtrinsics(extrinsicsColortoDepth, "extrinsics_color_to_depth.json");
}

void VideoRecorder::saveExtrinsics(rs2_extrinsics extrinsics, string filename) {

	ofstream extrinsicsFile;
	extrinsicsFile.open(this->baseDir + filename);
	extrinsicsFile << "{" << endl;
	extrinsicsFile << "    \"rotation_matrix\": [";
	for (auto i = 0; i < 3; ++i)
	{
		if (i != 0){
			extrinsicsFile << "                        ";
		}
		for (auto j = 0; j < 3; ++j)
		{	
			extrinsicsFile << extrinsics.rotation[j * 3 + i];

			if (i!=2) {
				if (j != 2) {
					extrinsicsFile << ",";
				}
				else {
					extrinsicsFile << "," << endl;
				}
			}
			else {
				if (j != 2) {
					extrinsicsFile << ",";
				}
				else {
					extrinsicsFile << "]," << endl;
				}
			}
		}
	}
	extrinsicsFile << "    \"translation_vector\": [";
	for (auto i = 0u; i < sizeof(extrinsics.translation) / sizeof(extrinsics.translation[0]); ++i) {
		if (i != 2) {
			extrinsicsFile << extrinsics.translation[i] << ",";
		}
		else {
			extrinsicsFile << extrinsics.translation[i] << "]" << endl;
		}

	}
	extrinsicsFile << "}" << endl;
	extrinsicsFile.close();
}

void VideoRecorder::saveIntrinsics(rs2_intrinsics intrinsics, string filename) {
	ofstream intrinsicsFile;
	intrinsicsFile.open(this->baseDir + filename);
	intrinsicsFile << "{" << endl;
	intrinsicsFile << "    \"coeffs\": [" << intrinsics.coeffs[0] << "," << endl;
	intrinsicsFile << "               " << intrinsics.coeffs[1] << "," << endl;
	intrinsicsFile << "               " << intrinsics.coeffs[2] << "," << endl;
	intrinsicsFile << "               " << intrinsics.coeffs[3] << "," << endl;
	intrinsicsFile << "               " << intrinsics.coeffs[4] << "]," << endl;
	intrinsicsFile << "    \"fx\": " << intrinsics.fx << "," << endl;
	intrinsicsFile << "    \"fy\": " << intrinsics.fy << "," << endl;
	intrinsicsFile << "    \"height\": " << intrinsics.height << "," << endl;
	intrinsicsFile << "    \"ppx\": " << intrinsics.ppx << "," << endl;
	intrinsicsFile << "    \"ppy\": " << intrinsics.ppy << "," << endl;
	intrinsicsFile << "    \"width\": " << intrinsics.width << endl;
	intrinsicsFile << "}" << endl;

	intrinsicsFile.close();
}

void VideoRecorder::writeIntrinsics() {

	auto videoStream = this->rsPipeline.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	rs2_intrinsics intrinsics = videoStream.get_intrinsics();
	this->saveIntrinsics(intrinsics,"intrinsics_depth.json");

	auto colorStream = this->rsPipeline.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	rs2_intrinsics intrinsicsColor = colorStream.get_intrinsics();
	this->saveIntrinsics(intrinsicsColor, "intrinsics_color.json");
}

void VideoRecorder::writeDepthDeviceInformation() {
	rs2::depth_sensor depthSensor = this->rsPLProfile.get_device().first<rs2::depth_sensor>();
	ofstream depthFile;
	depthFile.open(this->baseDir + "depth_parameters.json");
	depthFile << "{" << endl;
	depthFile << " \" depth_scale\":  " << depthSensor.get_depth_scale() << "," << endl;
	depthFile << "}" << endl;
	depthFile.close();
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

			if (verifyOptionSupport(depthSensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
				depthSensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0);
			}

			if (verifyOptionSupport(depthSensor, RS2_OPTION_EMITTER_ENABLED)) {
				depthSensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
			}
			if (verifyOptionSupport(depthSensor, RS2_OPTION_DEPTH_UNITS)) {
				depthSensor.set_option(RS2_OPTION_DEPTH_UNITS, 0.001);
			}
		}

		/*
		Turn on the auto-exposure and turn off white balance for RGB sensor.
		*/

		if (this->enableRGB) {
			rs2::color_sensor colorSensor = this->rsPLProfile.get_device().first<rs2::color_sensor>();

			if (verifyOptionSupport(colorSensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
				colorSensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0);
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

rs2::config VideoRecorder::createContext() { 
	rs2:config rsConfig;

	if (this->enableDepth) {
		rsConfig.enable_stream(RS2_STREAM_DEPTH, 0, 848, 480, RS2_FORMAT_Z16, this->Depth_FPS);
	}

	if (this->enableRGB) {
		rsConfig.enable_stream(RS2_STREAM_COLOR, 0, 1920, 1080, RS2_FORMAT_BGR8, this->RGB_FPS);
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


void VideoRecorder::setNewDepthROI(){
	// Here make the call to the sensor

	float x_scaling = (840.0f / 1080.0f);
	float y_scaling = (480.0f / 720.0f);

	rs2::region_of_interest roi;
	rs2::roi_sensor roi_sensor(this->rsPLProfile.get_device().first<rs2::depth_sensor>());
	roi = roi_sensor.get_region_of_interest();
	roi.min_x = int(this->depthROI.origin.x * x_scaling);
	roi.min_y = int(this->depthROI.origin.y * y_scaling);
	roi.max_x = int(this->depthROI.end.x * x_scaling);
	roi.max_y = int(this->depthROI.end.y * y_scaling);
	if (roi.max_y - roi.min_y >= 24 && roi.max_x - roi.min_x >= 48) {
		roi_sensor.set_region_of_interest(roi);
	}

}

// TODO: Change which exposure we are setting, either depth or color, not together.


	//if(auto_exposure_not_enabled)
	//{

	//	if (this->depthROI.hasUpdated) {
	//		this->setNewDepthROI();
	//		this->depthROI.hasUpdated = false;
	//	}


void VideoRecorder::verifySetUp() {
	rs2::align alignTo(RS2_STREAM_COLOR);

	rs2::frameset frameSet;

	rs2::frame colorFrame;
	rs2::frame depthFrame;
	std::chrono::high_resolution_clock Clock;
	std::chrono::duration<float> timeElapsed;
	auto startTime = Clock.now();
	timeElapsed = Clock.now() - startTime;

	int frame_count = 0;
	while (true) {

		timeElapsed = Clock.now() - startTime;

		frameSet = this->rsPipeline.wait_for_frames(10000);


		frameSet = alignTo.process(frameSet);
		colorFrame = frameSet.get_color_frame();
		depthFrame = frameSet.get_depth_frame();

		if (this->videoController.update(colorFrame, depthFrame) == 0) {
			break;
		}

		frame_count += 1;
	}
}


void VideoRecorder::recordVideo() {

	// ################## Necessary for raw recording. ################## //

	rs2::frame_queue depthFramesQueue(2);
	rs2::frame_queue colorFramesQueue(2);

	// Prepare camera and let auto-exposure settle.
	std::this_thread::sleep_for(std::chrono::seconds(5));

	// Depth colorization and alignment.
	rs2::align alignTo(RS2_STREAM_COLOR);

	// Information tracking
	int recordedFrameCount = 0;
	int maxFrames =  static_cast<int>(this->fullSessionLength * max(this->RGB_FPS, this->Depth_FPS));

	// Calculate maximum amount of frames
	int max_frames = static_cast<int>(this->individualVideoLength * static_cast<float>(this->videoCount) * max(this->RGB_FPS, this->Depth_FPS));
		
	// Create clock
	std::chrono::high_resolution_clock Clock;

	// Object for frames
	rs2::frameset frameSet;

	rs2::frame colorFrame;
	rs2::frame depthFrame;

	std::thread depthSavingThread(writeFrames, depthFramesQueue, "depth", this->depthDir, this->baseDir, this->videoCount, this->individualVideoLength, 6, this->minDepth, this->maxDepth);
	std::thread colorSavingThread(writeFrames, colorFramesQueue, "color", this->colorDir, this->baseDir, this->videoCount, this->individualVideoLength, this->RGB_FPS, this->minDepth, this->maxDepth);

	// Keep track of time in video.
	std::chrono::duration<float> timeElapsed;
	auto startTime = Clock.now();
	timeElapsed = Clock.now() - startTime;

	auto preview_key = (char)32;
	bool displayVideo = false;
	
	this->videoController.createCVWindow();

	cv::Mat depthImage;
	cv::Mat colorImage;

	//	################## Preview necessaries for depth. ################## //

	rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
	rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
	rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
	rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise

	auto min_depth = this->minDepth; // Given that we are recording an incubator.
	auto max_depth = this->maxDepth;  // Given that we are recording an incubator.

	rs2::disparity_transform depth_to_disparity(true);
	rs2::disparity_transform disparity_to_depth(false);
	rs2::colorizer color_filter;

	// filter settings
	thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, min_depth);
	thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, max_depth);
	color_filter.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 0);
	color_filter.set_option(RS2_OPTION_COLOR_SCHEME, 9.0f);		// Hue colorization
	color_filter.set_option(RS2_OPTION_MAX_DISTANCE, max_depth);
	color_filter.set_option(RS2_OPTION_MIN_DISTANCE, min_depth);


	// Record a video
	while (true) {

		timeElapsed = Clock.now() - startTime;

		if (timeElapsed.count() >= this->fullSessionLength) {
			break;
		}

		try {
			frameSet = this->rsPipeline.wait_for_frames(10000);

			if (recordedFrameCount % 5 == 0) {
				frameSet = alignTo.process(frameSet);
				colorFrame = frameSet.get_color_frame();
				depthFrame = frameSet.get_depth_frame();

				depthFramesQueue.enqueue(depthFrame);
				colorFramesQueue.enqueue(colorFrame);
			}
			else {
				colorFrame = frameSet.get_color_frame();
				colorFramesQueue.enqueue(colorFrame);
			}

			this->videoController.update(colorFrame, depthFrame);

			// Updating time loop and frames
			recordedFrameCount++;

			// Debug printing
			if (recordedFrameCount % 900 == 0) {
				std::cout << recordedFrameCount << " at " << timeElapsed.count() << " seconds." << endl;
			}
		}
		catch (const rs2::error& e) {
			std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
			std::cerr << "Camera did overheat with a temperature of: " << this->rsPipeline.get_active_profile().get_device().first<rs2::depth_sensor>().get_option(RS2_OPTION_ASIC_TEMPERATURE) << "C." << endl;
			std::cerr << "Program has unexpectedly exited." << endl << "Try moving the sensor further from the incubator." << endl;
			cv::destroyAllWindows();
			system("pause");
		}
	}

	cv::destroyAllWindows();
	std::cout << "Number of frames captured:" << recordedFrameCount << endl;
	std::cout << "Number of max possible frames:" << maxFrames << endl;

	colorSavingThread.join();
	depthSavingThread.join();

	return;
}