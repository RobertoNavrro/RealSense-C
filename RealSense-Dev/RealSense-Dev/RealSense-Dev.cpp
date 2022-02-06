// RealSense-Dev.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "VideoRecorder.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/base.hpp>
#include <iostream>
#include "Utilities.h"

using namespace std;
int main(){

    VideoRecorder recorder(0.5, 1.0, true, true);
    recorder.verifySetUp();
    recorder.recordVideo();
    recorder.stopPipeline();
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu
