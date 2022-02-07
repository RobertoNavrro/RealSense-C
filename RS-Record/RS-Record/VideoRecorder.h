#include <string>

#include <librealsense2/rs.hpp>

#ifndef VIDEORECORDER_H
#define VIDEORECORDER_H

using namespace std;
using namespace rs2;

class VideoRecorder
{
	private:

		const string parentDir = string("./SLAPI_Recordings/");
		string baseDir;
		string colorDir;
		string depthDir;
	
		rs2::pipeline rsPipeline;
		rs2::pipeline_profile rsPLProfile;

		const float RGB_FPS = 30;
		const float Depth_FPS = 30;
		float individualVideoLength;
		float fullSessionLength;
		int videoCount;

		bool showVideo;
		bool enableRGB;
		bool enableDepth;
		bool verifyOptionSupport(rs2::sensor, rs2_option);

		void calculateIndividualVidLength(float min);
		void determineOutputVideoCount();
		void calculateSessionLength(float minutes);
		void createDirectories();
		void setDirectories();
		rs2::config createContext();
		void startPipeline(rs2::config);
		void controlSensorSettings();

	public:
		VideoRecorder(float individualVideoLength, float fullSessionLength, bool enableRGB, bool enableDepth);
		void recordVideo();
		void stopPipeline();
		void verifySetUp();
};

#endif // !
