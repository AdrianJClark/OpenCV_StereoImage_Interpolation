#include "CaptureLibrary.h"
#include <iostream>

/* Video Object Constructors */
Video::Video(char *videoFile) {
	captureObj = cvCreateFileCapture(videoFile);
	if (!captureObj) {std::cerr << "Unable to load video file: " << videoFile << std::endl; exit(0); }

	captureWidth = (int)cvGetCaptureProperty(captureObj, CV_CAP_PROP_FRAME_WIDTH); 
	captureHeight = (int)cvGetCaptureProperty(captureObj, CV_CAP_PROP_FRAME_HEIGHT);	
	captureParams = 0;
	captureUndistort=false;
}

Video::Video(char *videoFile, char *parametersFile) {
	captureObj = cvCreateFileCapture(videoFile);
	if (!captureObj) { std::cerr << "Unable to load video file: " << videoFile << std::endl; exit(0); }

	if (!loadCaptureParams(parametersFile)) { std::cerr << "Unable to camera parameter file: " << parametersFile << std::endl; cvReleaseCapture(&captureObj); exit(0); }

	if (captureWidth==-1) captureWidth = (int)cvGetCaptureProperty(captureObj, CV_CAP_PROP_FRAME_WIDTH); 
	if (captureHeight==-1) captureHeight = (int)cvGetCaptureProperty(captureObj, CV_CAP_PROP_FRAME_HEIGHT);	
	captureUndistort=true;
}
