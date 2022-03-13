#include "VideoController.h"
#include "ROIHolder.h"
#include "Utilities.h"
#include "ControllingTypes.h"

void depthROICallback(int event, int x, int y, int flags, void* userdata) {

	ROIHolder* depthROI = (ROIHolder*)userdata;

	if (event == cv::EVENT_LBUTTONDOWN) {
		depthROI->setOriginPoint(x, y);
		depthROI->dragging = true;
	}

	else if (event == cv::EVENT_LBUTTONUP) {
		depthROI->calculateDimensions(x, y);
		depthROI->dragging = false;
		depthROI->hasUpdated = true;
		depthROI->print();
	}

	if (depthROI->dragging) {
		depthROI->setEndPoint(x, y);
	}
}

VideoController::VideoController() {
	this->windowName = "Preview";
}

VideoController::VideoController(rs2::sensor& colorSensor, rs2::sensor& depthSensor, string windowName) {
	this->colorSensor = colorSensor.as<rs2::color_sensor>();
	this->depthSensor = depthSensor.as<rs2::depth_sensor>();
	this->windowName = windowName;

	this->depth_exposure_value = (int)this->depthSensor.get_option(rs2_option::RS2_OPTION_EXPOSURE);
	this->rgb_exposure_value = (int)this->colorSensor.get_option(rs2_option::RS2_OPTION_EXPOSURE);

	this->setFilterSettings();
	this->createCVWindow();
}

void VideoController::createCVWindow() {
	cv::namedWindow(this->windowName, WINDOW_AUTOSIZE);
	this->is_video_destroyed = false;
	this->is_showing_video = true;
	//cv::setMouseCallback(this->windowName, depthROICallback, (void*)&this->depthROI);
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
		cv::destroyWindow(this->windowName);
		this->is_video_destroyed = true;
		return;
	}

	if (keypress == this->tab_key) {
		tab_hit = true;
	}

	if (keypress == this->preview_toggle) {
		this->is_showing_video = !this->is_showing_video;

		if (!this->is_showing_video) {
			cv::imshow(this->windowName, this->defaultImage);
		}

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

	if (this->is_showing_video) {

		depthFrame = this->depth_to_disparity.process(depthFrame);
		depthFrame = this->spat_filter.process(depthFrame);
		depthFrame = this->temp_filter.process(depthFrame);
		depthFrame = this->disparity_to_depth.process(depthFrame);
		depthFrame = this->color_filter.process(depthFrame);

		cv::Mat colorImage = frame_to_mat(colorFrame);
		cv::Mat depthImage = frame_to_mat(depthFrame);

		cv::Mat color_image;
		cv::Mat depth_image;

		cv::resize(colorImage, color_image, cv::Size(1080, 720), cv::INTER_LINEAR);
		cv::resize(depthImage, depth_image, cv::Size(1080, 720), cv::INTER_LINEAR);

		if (!color_image.empty() && !depth_image.empty()) {

			cv::Mat blendedImage;

			cv::addWeighted(color_image, 1, depth_image, 0.5, 0.0, blendedImage);


			/*if (this->auto_exposure_is_enabled) {
				this->depthROI.drawROI(blendedImage);
			}*/

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

int VideoController::update( rs2::frame& colorFrame,  rs2::frame& depthFrame) {
	if (this->is_video_destroyed) {
		return 0;
	}
	this->showVideo(colorFrame, depthFrame);
	this->handleInput();

	return 1;
}

