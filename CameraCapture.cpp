#include "CaptureLibrary.h"
#include <iostream>

void Camera::init(int cameraIndex, CvSize imgSize, char* parametersFile) {
	vi = new videoInput();
	vi->setVerbose(false); vi->setUseCallback(true);
	vi->setupDevice(cameraIndex, imgSize.width, imgSize.height);

	if (parametersFile) { loadCaptureParams(parametersFile); captureUndistort=true; } else { captureParams = 0; captureUndistort=false;}

	captureWidth = vi->getWidth(cameraIndex); captureHeight = vi->getHeight(cameraIndex);
	camIndex = cameraIndex;
}