#include <stdio.h>
#include <string>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include "opencv2/videoio.hpp"
#include <exception>
#include <opencv-3.4/modules/videoio/include/opencv2/videoio/videoio_c.h>

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
    float fps) {

    cout << "Writing in directory:" << directory << "for type:" << imageType << endl;
    int videoID = 1;
    //Define video output
    auto fourcc = CV_FOURCC('m', 'p', '4', 'v');

    //Define file name
    string filename = directory + to_string(videoID) + ".mp4";

    //Define Size
    cv::Size resolution = cv::Size(1920, 1080);

    //Define writer
    cv::VideoWriter writer(filename, fourcc, static_cast<double>(fps), resolution, true);

    cv::Mat currentFrame;
    rs2::frame frame;

    int frameCount = 0;

    int maxVideoFrames = static_cast<int>(individualVideoLength * fps);

    cout << "Maximum frames per video:" << maxVideoFrames << endl;

    //Filtering necessary for depth

    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise

    float min_depth = 0.20f; // Given that we are recording an incubator.
    float max_depth = 7.0f;  // Given that we are recording an incubator.

    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);
    rs2::colorizer color_filter;

    // filter settings
    thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, min_depth);
    thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, max_depth);
    color_filter.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 0);
    color_filter.set_option(RS2_OPTION_COLOR_SCHEME, 9.0f);		// Hue colorization
    color_filter.set_option(RS2_OPTION_MAX_DISTANCE, max_depth);
    color_filter.set_option(RS2_OPTION_MIN_DISTANCE, min_depth);

    // Create clock
    std::chrono::high_resolution_clock Clock;

    // Keep track of time in video.
    std::chrono::duration<float> timeElapsed;
    auto startTime = Clock.now();
    timeElapsed = Clock.now() - startTime;


    while (true) {

        timeElapsed = Clock.now() - startTime;
        
        if (queue.capacity() == queue.size()) {
            cout << "Queue for " << imageType << "is full. Frame dropping might occur." << endl;
        }

        if (individualVideoLength <= timeElapsed.count()) {

            videoID++;
            writer.release();

            filename = directory + to_string(videoID) + ".mp4";

            writer = cv::VideoWriter(filename, fourcc, static_cast<double>(fps), resolution);

            std::cout << "Starting to write video " << videoID << " of type: " << imageType <<  "." << endl;

            startTime = Clock.now();
        }

        try {
            frame = queue.wait_for_frame(30000); // we wait at most 15 seconds otherwise we know the frames have ended and we can stop.
            frameCount += 1;

            if (imageType == "depth") {
                frame = thr_filter.process(frame);
                frame = depth_to_disparity.process(frame);
                frame = spat_filter.process(frame);
                frame = temp_filter.process(frame);
                frame = disparity_to_depth.process(frame);
                frame = color_filter.process(frame);
            }

            currentFrame = frame_to_mat(frame);
            writer.write(currentFrame);
        }
        catch (const cv::Exception& e) {
            std::cout << "Exception caught while writing:" << filename << "Exception msg:" << e.what() << std::endl;
            break;
        }
        catch (const rs2::error& e) {
            std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
            std::cerr << "Exiting save thread for " << imageType << endl;
            break;
        }
    }
    writer.release();
    writer.~VideoWriter();
    return;
}


bool isPathExist(const std::string& s)
{
    struct stat buffer;
    return (stat(s.c_str(), &buffer) == 0);
}