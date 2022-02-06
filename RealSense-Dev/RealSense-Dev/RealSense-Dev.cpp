// RealSense-Dev.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "VideoRecorder.h"

using namespace std;
int main()
{
	VideoRecorder recorder = VideoRecorder(1.0,1.0,true,true);
	recorder.recordVideo();
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu
