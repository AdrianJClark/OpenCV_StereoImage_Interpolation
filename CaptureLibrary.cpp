#include "CaptureLibrary.h"
#include <iostream>

/* Camera Destructor */
Capture::~Capture() {
	if (captureParams) cvReleaseMat(&captureParams);
	if (captureDistortion) cvReleaseMat(&captureDistortion);
	if (mDistortX) cvReleaseMat(&mDistortX);
	if (mDistortY) cvReleaseMat(&mDistortY);
}

/* Get a frame from the camera */
IplImage* Capture::getFrame() {
		IplImage* newFrame = cvCreateImage(cvSize(captureWidth, captureHeight), IPL_DEPTH_8U, 3);
		if (vi) newFrame->imageData = (char*)vi->getPixels(camIndex, false, true);
		
		if (captureUndistort) {
			IplImage *unDistortedFrame = cvCreateImage(cvGetSize(newFrame), newFrame->depth, newFrame->nChannels);
			cvRemap( newFrame, unDistortedFrame, mDistortX, mDistortY);
			cvReleaseImage(&newFrame); newFrame = unDistortedFrame;
		}

		return newFrame;
}

/* Load the Camera Parameters */
bool Capture::loadCaptureParams(char *filename) {
	CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );
	if (fs==0) return false; 

	CvFileNode* fileparams;
	//Read the Image Width
	fileparams = cvGetFileNodeByName( fs, NULL, "image_width" );
	captureWidth = cvReadInt(fileparams,320);
	//Read the Image Height
	fileparams = cvGetFileNodeByName( fs, NULL, "image_height" );
	captureHeight = cvReadInt(fileparams,240);
	//Read the Camera Parameters
	fileparams = cvGetFileNodeByName( fs, NULL, "M" );
	captureParams = (CvMat*)cvRead( fs, fileparams );
	principalX = captureParams->data.db[2]; principalY = captureParams->data.db[5];
	//Read the Camera Distortion 
	fileparams = cvGetFileNodeByName( fs, NULL, "D" );
	captureDistortion = (CvMat*)cvRead( fs, fileparams );
	cvReleaseFileStorage( &fs );

	//Initialize Undistortion Maps
    mDistortX = cvCreateMat(captureHeight, captureWidth, CV_32F);
    mDistortY = cvCreateMat(captureHeight, captureWidth, CV_32F);
	cvInitUndistortMap(captureParams, captureDistortion, mDistortX, mDistortY);

	return true;
}

/* Return the Camera Parameters */
CvMat* Capture::getCaptureParameters() {
	//If the image has been undistorted, update the principal point to be the center of the image
	if (captureUndistort) { 
		captureParams->data.db[2] = captureWidth/2.0; captureParams->data.db[5] = captureHeight/2.0; 
	} else {
		captureParams->data.db[2] = principalX; captureParams->data.db[5] = principalY; 
	}
	
	return captureParams;
}

/* Return the Camera Distortion */
CvMat* Capture::getCaptureDistortion() {
	//If the image has been undistorted, return 0
	if (captureUndistort) 	return 0;
	return captureDistortion;
}
