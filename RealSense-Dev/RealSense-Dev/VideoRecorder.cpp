#include "VideoRecorder.h"

#include "librealsense2/rs.hpp"
#include <assert.h>
#include<direct.h>

#include <iostream>
#include <filesystem>
#include <vector>

#include "Utilities.h"

using namespace rs2;
using namespace std;


VideoRecorder::VideoRecorder (int individualVideoLength,int fullSessionLength, bool enableRGB, bool enableDepth) {
	this->enableDepth = enableDepth;
	this->enableRGB = enableRGB;

	this->calculateSessionLength(fullSessionLength);
	this->calculateIndividualVidLength(individualVideoLength);
	this->determineOutputVideoCount();
	this->setDirectories();
	this->createDirectories();
}


void VideoRecorder::calculateIndividualVidLength(int min) {
	if (min <= 0 ) {
		min = 1;
	}
	individualVideoLength = min * 60;
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
	if (minutes < 1) {
		minutes = 1;
	}
	fullSessionLength = int(minutes * 60);
}

bool VideoRecorder::createDirectories() {
	vector<string> directories = { this->parentDir,
								   this->baseDir,
								   this->colorDir,
								   this->depthDir};

	int success = 1;

	for(string dir : directories)
	{
		if (success == 0) {
			return false;
		}

		if (isPathExist(dir)) {
			continue;
		}

		else {
			success = _mkdir(dir.c_str());
		}
	}

	return true;
}

void VideoRecorder::setDirectories() {
	std::time_t now = std::time(0);
	tm timeinfo[80];
	char time_buffer[80];
	
	localtime_s(timeinfo, &now);

	strftime(time_buffer, 80, "%Y_%m_%d-%H_%M", timeinfo);

	cout << time_buffer << endl;

	this->baseDir = this->parentDir +  "/output_" + string(time_buffer);
	this->colorDir = this->baseDir + "/color";
	this->depthDir = this->baseDir + "/depth";
}
