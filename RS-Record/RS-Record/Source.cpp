//

#include "opencv2/videoio.hpp"
#include <opencv2/opencv.hpp> 
#include <opencv-3.4/modules/videoio/include/opencv2/videoio/videoio_c.h>

using namespace std;
int main() {
    auto fourcc = CV_FOURCC('m', 'p', '4', 'v');

    //Define file name
    string filename = "test.mp4";

    //Define Size
    cv::Size resolution = cv::Size(1920, 1080);

	cv::VideoWriter writer(filename, fourcc, static_cast<float>(30), resolution, true);
}
