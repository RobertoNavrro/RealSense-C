#include <string>
#include "opencv2/opencv.hpp"
#include <librealsense2/rs.hpp>
#include <concurrent_queue.h>

#ifndef UTILITIES_H
#define UTILITIES_H

bool isPathExist(const std::string& filename);
cv::Mat frame_to_mat(const rs2::frame& f);
void writeFrames(rs2::frame_queue queue, std::string imageType, 
				 std::string directory, std::string baseDirectory, 
				 int videoCount, float individualVideoLength, float fps, float min_depth, float max_depth);

#endif // !