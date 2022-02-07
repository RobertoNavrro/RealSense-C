#include <stdio.h>
#include <string>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include "opencv2/videoio.hpp"
#include <opencv-3.4/modules/videoio/include/opencv2/videoio/videoio_c.h>
#include <exception>

using namespace cv;
using namespace rs2;
using namespace std;

// Convert rs2::frame to cv::Mat
cv::Mat frame_to_mat(const rs2::frame& f)
{

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}


void writeFrames(rs2::frame_queue queue,
                 std::string imageType,
                 std::string directory,
                 std::string baseDirectory,
                 int videoCount,
                 float individualVideoLength,
                 int fps) {

    cout << "Writing in directory:" << directory << "for type:" << imageType << endl;
    int videoID = 1;
    //Define video output
    auto fourcc = CV_FOURCC('m', 'p', '4', 'v');

    //Define file name
    string filename = directory + to_string(videoID) + ".mp4";
    
    //Define Size
    cv::Size resolution = cv::Size(1920, 1080);

    //Define writer
    cv::VideoWriter writer (filename, fourcc, static_cast<float>(15), resolution, true);

    cv::Mat currentFrame;
    rs2::frame frame;

    int frameCount = 0;

    int maxVideoFrames = individualVideoLength * fps;

    cout << "Maximum frames per video:" << maxVideoFrames << endl;

    try {
        while (true) {
            frame = queue.wait_for_frame(10000);
            currentFrame = frame_to_mat(frame);
            frameCount += 1;
            if (frameCount >= maxVideoFrames) {
                std::cout << "Done recording video" << videoID << ".Type: " << imageType <<endl;
                videoID += 1;
                writer.release();
                if (videoID > videoCount) {
                    break;
                }
                else {
                    filename = directory + to_string(videoID) + ".mp4";
                    writer = cv::VideoWriter(filename, fourcc, static_cast< float >(fps), resolution);
                    std::cout << "Starting to write for" << videoID << ".Type: " << imageType << endl;
                }
                frameCount = 0;
            }

            if (imageType == "depth"){
                cout << queue.size() << endl;
                cv::convertScaleAbs(currentFrame, currentFrame, 0.03);
                cv::applyColorMap(currentFrame, currentFrame, COLORMAP_VIRIDIS);
            }

            writer.write(currentFrame);
        }
    }
    catch (cv::Exception& e) {
        std::cout << "Exception caught while writing:" << filename << "Exception msg:" << e.what() << std::endl;
        writer.release();
        return;
    }
    catch (const rs2::error& e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        writer.release();
    }
    writer.release();
    return;
}


bool isPathExist(const std::string& s)
{
    struct stat buffer;
    return (stat(s.c_str(), &buffer) == 0);
}