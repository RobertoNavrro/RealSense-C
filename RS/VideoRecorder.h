#include <string>

#include <librealsense2/rs.hpp>
#include "ROIHolder.h"

#ifndef VIDEORECORDER_H
#define VIDEORECORDER_H

using namespace std;
using namespace rs2;

class VideoRecorder
{
	private:

		const string parentDir = string("C:/Users/Neonatology Research/Documents/SLAPI data/VIDEO_RECORDINGS/");
		string baseDir;
		string colorDir;
		string depthDir;
		ROIHolder depthROI;
	
		rs2::pipeline rsPipeline;
		rs2::pipeline_profile rsPLProfile;

		const float RGB_FPS = 30;
		const float Depth_FPS = 30;
		float minDepth = 0.19f;
		float maxDepth = 7.0f;
		float individualVideoLength;
		float fullSessionLength;
		int videoCount;

		bool showVideo;
		bool manuallyAdjust;
		bool enableRGB = true;
		bool enableDepth = true;
		bool auto_exposure_is_enabled = true;
		int rgb_exposure_value;
		int depth_exposure_value;
		bool exposure_switched = false;
		bool verifyOptionSupport(rs2::sensor, rs2_option);

		void calculateIndividualVidLength(float min);
		void determineOutputVideoCount();
		void calculateSessionLength(float minutes);
		void createDirectories();
		void setDirectories();
		rs2::config createContext();
		void startPipeline(rs2::config);
		void controlSensorSettings();
		void writeIntrinsics();
		void writeExtrinsics();
		void saveIntrinsics(rs2_intrinsics intrinsics, string filename);
		void saveExtrinsics(rs2_extrinsics extrinsics, string filename);
		void writeDepthDeviceInformation();
		void setNewDepthROI();
		int adjustVideo(const rs2::frame& colorFrame, const rs2::frame& depthFrame, string windowName);
		void setDepthROIDefault(int width, int height);
	public:
		VideoRecorder(float individualVideoLength, float fullSessionLength, bool enableRGB, bool enableDepth);
		void recordVideo();
		void stopPipeline();
		void verifySetUp();
};

#endif // !
