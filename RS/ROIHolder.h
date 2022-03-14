#include <opencv2/core/types.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>

#ifndef ROIHOLDER_H
#define ROIHOLDER_H

using namespace std;
using namespace cv;

struct ROIHolder {

	int width = 0, height = 0;
	cv::Point origin = cv::Point(0, 0);
	cv::Point end = cv::Point(0, 0);
	bool hasUpdated = false;
	bool dragging = false;

	void calculateDimensions(int x, int y) {

		if (x < origin.x) {
			width = (origin.x - x);
			origin.x = x;
			end.x = x + width;
		}
		else {
			width = x - origin.x;
		}

		if (y < origin.y) {
			height = (origin.y - y);
			origin.y = y;
			end.y = y + height;
		}
		else {
			height = y - origin.y;
		}
	}

	void drawROI(cv::Mat& image) {
		if (origin.x != 0 && origin.y != 0 && height != 0 && width != 0) {
			cv::rectangle(image, origin, end, cv::Scalar(255, 0, 255));
			cv::rectangle(image, origin - cv::Point(1, 1), end + cv::Point(1, 1), cv::Scalar(0, 255, 255));
		}
	}

	void setOriginPoint(int x, int y) {
		origin.x = x;
		origin.y = y;
		std::cout << "Origin X:" << origin.x << std::endl;
		std::cout << "Origin Y:" << origin.y << std::endl;
	}

	void setEndPoint(int x, int y) {
		end.x = x;
		end.y = y;
		std::cout << "End X:" << end.x << std::endl;
		std::cout << "End Y:" << end.y << std::endl;
	}

	void print() {
		std::cout << "Origin X:" << origin.x << std::endl;
		std::cout << "Origin Y:" << origin.y << std::endl;
		std::cout << "Width:" << width << std::endl;
		std::cout << "Height:" << height << std::endl;
		std::cout << "End X:" << end.x << std::endl;
		std::cout << "End Y:" << end.y << std::endl;
		std::cout << "hasUpdated:" << hasUpdated << std::endl;
	}
};

#endif // !
