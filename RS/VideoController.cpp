#include "VideoController.h"
#include "ROIHolder.h"
#include "Utilities.h"
#include "ControllingTypes.h"

void ROICallback(int event, int x, int y, int flags, void* userdata) {

	ROIHolder* ROI = (ROIHolder*)userdata;

	if (event == cv::EVENT_LBUTTONDOWN) {
		ROI->setOriginPoint(x, y);
		ROI->dragging = true;
	}

	else if (event == cv::EVENT_LBUTTONUP) {
		ROI->calculateDimensions(x, y);
		ROI->dragging = false;
		ROI->hasUpdated = true;
		ROI->print();
	}

	if (ROI->dragging) {
		ROI->setEndPoint(x, y);
	}
}

VideoController::VideoController() {
}

VideoController::VideoController(rs2::sensor& colorSensor, rs2::sensor& depthSensor, string windowName) {
	this->colorSensor = colorSensor.as<rs2::color_sensor>();
	this->depthSensor = depthSensor.as<rs2::depth_sensor>();
	this->windowName = windowName;

	this->depth_exposure_value = (int)this->depthSensor.get_option(rs2_option::RS2_OPTION_EXPOSURE);
	this->rgb_exposure_value = (int)this->colorSensor.get_option(rs2_option::RS2_OPTION_EXPOSURE);

	this->setFilterSettings();
	this->setROIDefault(this->displayWidth, this->displayHeight);
	this->createCVWindow();
}

void VideoController::createCVWindow() {
	cv::namedWindow(this->windowName, WINDOW_AUTOSIZE);
	this->is_showing_preview = true;
	cv::setMouseCallback(this->windowName, ROICallback, (void*)&this->ROI);
}

void VideoController::pauseCVWindow() {
	cv::resizeWindow(this->windowName, 25, 25);
	this->is_showing_preview = false;
	cv::setMouseCallback(this->windowName, NULL, NULL);
}

void VideoController::unpauseCVWindow() {
	cv::resizeWindow(this->windowName, this->displayWidth, this->displayHeight);
	this->is_showing_preview = true;
	cv::setMouseCallback(this->windowName, ROICallback, (void*)&this->ROI);
}

void VideoController::destroyCVWindow() {
	cv::destroyWindow(this->windowName);
	this->is_showing_preview = false;
}

void VideoController::setFilterSettings() {
	this->thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, this->minDepth);
	this->thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, this->maxDepth);
	this->color_filter.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 0);
	this->color_filter.set_option(RS2_OPTION_COLOR_SCHEME, 9.0f);
	this->color_filter.set_option(RS2_OPTION_MAX_DISTANCE, this->maxDepth);
	this->color_filter.set_option(RS2_OPTION_MIN_DISTANCE, this->minDepth);
}


void VideoController::controlDepthExposure() {

	if (!this->is_depth_auto_exposure_enabled) {

		depthSensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0);
		this->is_depth_auto_exposure_enabled = true;

	}
	else {
		this->depth_exposure_value = (int)this->depthSensor.get_option(rs2_option::RS2_OPTION_EXPOSURE);

		this->depthSensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0);

		this->depthSensor.set_option(RS2_OPTION_EXPOSURE, this->depth_exposure_value);

		this->is_depth_auto_exposure_enabled = false;
	}
}

void VideoController::controlColorExposure() {
	if (!this->is_rgb_auto_exposure_enabled) {

		this->colorSensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0);

		this->is_rgb_auto_exposure_enabled = true;
	}

	else {
		this->rgb_exposure_value = (int)this->colorSensor.get_option(rs2_option::RS2_OPTION_EXPOSURE);

		this->colorSensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0);

		this->colorSensor.set_option(RS2_OPTION_EXPOSURE, this->rgb_exposure_value);

		this->is_rgb_auto_exposure_enabled = false;
	}
}

void VideoController::handleInput() {
	auto keypress = cv::waitKey(5);
	bool tab_hit = false;

	if (keypress == this->esc_key) {
		//std::cout << "Pressed esc" << std::endl;
		if (this->is_showing_preview) {
			this->pauseCVWindow();
		}
		else {
			this->unpauseCVWindow();
		}
		return;
	}

	if (keypress == this->tab_key) {
		//std::cout << "Pressed tab" << std::endl;
		tab_hit = true;
	}

	switch (this->controlling_mode) {
		case controlling_types::automatic:
			if (tab_hit) {
				this->controlColorExposure();
				this->controlDepthExposure();
				this->controlling_mode = controlling_types::color;
			}

			this->depth_exposure_value = (int)this->depthSensor.get_option(rs2_option::RS2_OPTION_EXPOSURE);
			this->rgb_exposure_value = (int)this->colorSensor.get_option(rs2_option::RS2_OPTION_EXPOSURE);

			break;

		case controlling_types::color:
			if (tab_hit) {
				this->controlColorExposure();
				this->controlling_mode = controlling_types::depth;
			}

			if (keypress == this->increase_exposure){
				this->rgb_exposure_value += this->rgb_exposure_step_s;
				this->colorSensor.set_option(RS2_OPTION_EXPOSURE, this->rgb_exposure_value);
				cout << "Increasing color exposure" << endl;
			}
			
			if (keypress == this->decrease_exposure){
				this->rgb_exposure_value -= this->rgb_exposure_step_s;
				this->colorSensor.set_option(RS2_OPTION_EXPOSURE, this->rgb_exposure_value);
				cout << "Decreasing color exposure" << endl;
			}

			break;

		case controlling_types::depth:

			if (tab_hit) {
				this->controlDepthExposure();
				this->controlling_mode = controlling_types::automatic;
			}

			if (keypress == this->increase_exposure){
				this->depth_exposure_value += this->depth_exposure_step_s;
				this->depthSensor.set_option(RS2_OPTION_EXPOSURE, this->depth_exposure_value);
				cout << "Increasing depth exposure" << endl;
			}

			if (keypress == this->decrease_exposure) {
				this->depth_exposure_value -= this->depth_exposure_step_s;
				this->depthSensor.set_option(RS2_OPTION_EXPOSURE, this->depth_exposure_value);
				cout << "Decreasing depth exposure" << endl;
			}

			break;
	}

	return;
}

void VideoController::showVideo(rs2::frame& colorFrame, rs2::frame& depthFrame) {

	if (this->is_showing_preview) {
		depthFrame = this->depth_to_disparity.process(depthFrame);
		depthFrame = this->spat_filter.process(depthFrame);
		depthFrame = this->temp_filter.process(depthFrame);
		depthFrame = this->disparity_to_depth.process(depthFrame);
		depthFrame = this->color_filter.process(depthFrame);

		cv::Mat colorImage = frame_to_mat(colorFrame);
		cv::Mat depthImage = frame_to_mat(depthFrame);

		cv::Mat color_image;
		cv::Mat depth_image;

		cv::resize(colorImage, color_image, cv::Size(this->displayWidth, this->displayHeight), cv::INTER_LINEAR);
		cv::resize(depthImage, depth_image, cv::Size(this->displayWidth, this->displayHeight), cv::INTER_LINEAR);

		if (!color_image.empty() && !depth_image.empty()) {

			cv::Mat blendedImage;

			cv::addWeighted(color_image, 1, depth_image, 0.5, 0.0, blendedImage);

			this->ROI.drawROI(blendedImage);

			switch (this->controlling_mode) {
				case controlling_types::automatic:
					cv::putText(blendedImage, "Manual Mode: OFF", cv::Point(50, 50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
					cv::putText(blendedImage, "Manual Mode: OFF", cv::Point(51, 51), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));
					cv::putText(blendedImage, "Depth Exposure:" + to_string(this->depth_exposure_value), cv::Point(50, 75), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
					cv::putText(blendedImage, "Color Exposure:" + to_string(this->rgb_exposure_value), cv::Point(50, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
					cv::putText(blendedImage, "Depth Exposure:" + to_string(this->depth_exposure_value), cv::Point(51, 76), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));
					cv::putText(blendedImage, "Color Exposure:" + to_string(this->rgb_exposure_value), cv::Point(51, 101), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));
					break;
				case controlling_types::color:
					cv::putText(blendedImage, "Manual Mode: COLOR", cv::Point(50, 50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
					cv::putText(blendedImage, "Manual Mode: COLOR", cv::Point(51, 51), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));
					cv::putText(blendedImage, "Depth Exposure:" + to_string(this->depth_exposure_value), cv::Point(50, 75), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
					cv::putText(blendedImage, "Color Exposure:" + to_string(this->rgb_exposure_value), cv::Point(50, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
					cv::putText(blendedImage, "Depth Exposure:" + to_string(this->depth_exposure_value), cv::Point(51, 76), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));
					cv::putText(blendedImage, "Color Exposure:" + to_string(this->rgb_exposure_value), cv::Point(51, 101), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255),2);
					break;
				case controlling_types::depth:
					cv::putText(blendedImage, "Manual Mode: DEPTH", cv::Point(50, 50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
					cv::putText(blendedImage, "Manual Mode: DEPTH", cv::Point(51, 51), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));
					cv::putText(blendedImage, "Depth Exposure:" + to_string(this->depth_exposure_value), cv::Point(50, 75), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
					cv::putText(blendedImage, "Color Exposure:" + to_string(this->rgb_exposure_value), cv::Point(50, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
					cv::putText(blendedImage, "Depth Exposure:" + to_string(this->depth_exposure_value), cv::Point(51, 76), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255),2);
					cv::putText(blendedImage, "Color Exposure:" + to_string(this->rgb_exposure_value), cv::Point(51, 101), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));
					break;
			}
			
			cv::imshow(this->windowName, blendedImage);
		}
	}

}

void VideoController::update(rs2::frame& colorFrame, rs2::frame& depthFrame) {
	this->handleInput();
	if (this->is_showing_preview) {
		this->showVideo(colorFrame, depthFrame);
	}

	if (this->ROI.hasUpdated) {
		this->setNewDepthROI();
		this->setNewRgbROI();
		this->ROI.hasUpdated = false;
	}
}

void VideoController::setROIDefault(int width, int height) {
	int origin_x, origin_y;
	int roi_width, roi_height;

	roi_width = int(0.5 * width);
	roi_height = int(0.5 * height);

	this->ROI.width = roi_width;
	this->ROI.height = roi_height;

	this->ROI.setOriginPoint(int(0.25 * width), int(0.25 * height));
	this->ROI.setEndPoint(this->ROI.origin.x + roi_width, this->ROI.origin.y + roi_height);

	this->ROI.hasUpdated = true;

}

void VideoController::setNewDepthROI() {
	// Here make the call to the sensor

	float x_scaling = (840.0f / 1080.0f);
	float y_scaling = (480.0f / 720.0f);

	rs2::region_of_interest roi;
	rs2::roi_sensor roi_sensor(this->depthSensor);
	roi = roi_sensor.get_region_of_interest();
	roi.min_x = int(this->ROI.origin.x * x_scaling);
	roi.min_y = int(this->ROI.origin.y * y_scaling);
	roi.max_x = int(this->ROI.end.x * x_scaling);
	roi.max_y = int(this->ROI.end.y * y_scaling);
	if (roi.max_y - roi.min_y >= 24 && roi.max_x - roi.min_x >= 48) {
		roi_sensor.set_region_of_interest(roi);
	}

	std::cout << "RoI depth:" << std::endl;
	std::cout << "Min_x:" << roi.min_x << std::endl;
	std::cout << "Min_y:" << roi.min_y << std::endl;
	std::cout << "Max_x:" << roi.max_x << std::endl;
	std::cout << "Max_y:" << roi.max_y << std::endl;
}

void VideoController::setNewRgbROI() {
	// Here make the call to the sensor

	float x_scaling = (1920.0f / 1080.0f);
	float y_scaling = (1080.0f / 720.0f);

	rs2::region_of_interest roi;
	rs2::roi_sensor roi_sensor(this->colorSensor);
	roi = roi_sensor.get_region_of_interest();
	roi.min_x = int(this->ROI.origin.x * x_scaling);
	roi.min_y = int(this->ROI.origin.y * y_scaling);
	roi.max_x = int(this->ROI.end.x * x_scaling);
	roi.max_y = int(this->ROI.end.y * y_scaling);
	if (roi.max_y - roi.min_y >= 24 && roi.max_x - roi.min_x >= 48) {
		roi_sensor.set_region_of_interest(roi);
	}

	std::cout << "RoI RGB:" << std::endl;
	std::cout << "Min_x:" << roi.min_x << std::endl;
	std::cout << "Min_y:" << roi.min_y << std::endl;
	std::cout << "Max_x:" << roi.max_x << std::endl;
	std::cout << "Max_y:" << roi.max_y << std::endl;
}



ROIHolder VideoController::getROI() {
	return this->ROI;
}

bool VideoController::getShowingPreview() {
	return this->is_showing_preview;
}

void VideoController::setROIUpdated(bool updated) {
	this->ROI.hasUpdated = updated;
}
