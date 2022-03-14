#pragma once
#include <opencv2/core/types.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <librealsense2/rs.hpp>
#include <iostream>
#include <string>
#include "ROIHolder.h"
#include "ControllingTypes.h"

#ifndef VIDEOCONTROLLER_H
#define VIDEOCONTROLLER_H

using namespace std;
using namespace cv;


class VideoController
{

private:
	rs2::sensor colorSensor;
	rs2::sensor depthSensor; //MAYBE THIS IS NOT THE THING

	// Control Keys for sensor controllers
	char esc_key = (char)27;
	char enter_key = (char)13; // Enter, sets input
	char increase_exposure = (char)61; //+ key, for exposure
	char decrease_exposure = (char)45; //- key for exposure
	char tab_key = (char)9; // Change between manual and automatic mode.
	char control_depth_key = (char)100;
	char control_rgb_key = (char)99;

	bool is_depth_auto_exposure_enabled = true;
	bool is_rgb_auto_exposure_enabled = true;
	bool is_showing_preview = true;

	int rgb_exposure_value;
	int depth_exposure_value;

	int rgb_exposure_step_s = 30;
	int depth_exposure_step_s = 500;

	float minDepth = 0.19f;
	float maxDepth = 7.0f;

	int displayWidth = 1080;
	int displayHeight = 720; 

	// The RoI in the visible in the video preview
	ROIHolder ROI;
	void setROIDefault(int width, int height);
	

	cv::Mat defaultImage = cv::Mat(720, 1080, CV_8UC3, cv::Scalar(0, 0, 0));

	controlling_types controlling_mode = controlling_types::automatic;

	rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
	rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
	rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
	rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise

	rs2::disparity_transform depth_to_disparity = rs2::disparity_transform(true);
	rs2::disparity_transform disparity_to_depth = rs2::disparity_transform(false);
	rs2::colorizer color_filter;

public:
	VideoController();
	VideoController(rs2::sensor& colorSensor, rs2::sensor& depthSensor, string windowName);
	void handleInput();
	void setFilterSettings();
	void controlDepthExposure();
	void controlColorExposure();
	void createCVWindow();
	void pauseCVWindow();
	void unpauseCVWindow();
	void destroyCVWindow();
    void update(rs2::frame& colorFrame, rs2::frame& depthFrame);
	void showVideo(rs2::frame& colorFrame, rs2::frame& depthFrame);
	ROIHolder getROI();
	bool getShowingPreview();
	void setROIUpdated(bool updated);
	void setNewDepthROI();
	void setNewRgbROI();
	string windowName;
};

#endif //!

