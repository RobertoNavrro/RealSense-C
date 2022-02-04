#include <string>

#ifndef VIDEORECORDER_H
#define VIDEORECORDER_H

using namespace std;

class VideoRecorder
{
	private:
		string baseDir;
		string colorDir;
		string depthDir;
		string parentDir = string("./SLAPI_Recordings");

		int RGB_FPS;
		int Depth_FPS;
		int individualVideoLength;
		int fullSessionLength;
		int videoCount;
		bool showVideo;

		bool enableRGB;
		bool enableDepth;

	public:
		VideoRecorder(int individualVideoLength, int fullSessionLength, bool enableRGB, bool enableDepth);
		void calculateIndividualVidLength(int min);
		void determineOutputVideoCount();
		void calculateSessionLength(float minutes);
		bool createDirectories();
		void setDirectories();
};

#endif // !
