#ifndef CAPTURELIBRARY_H
#define CAPTURELIBRARY_H

#include <cv.h>
#include <highgui.h>
#include "videoInput.h"

class Capture {
public:
	Capture() { vi=0; camIndex=0; captureParams=captureDistortion=mDistortX=mDistortY=0;}
	~Capture();
	IplImage* getFrame();
	CvMat* getCaptureParameters();
	CvMat* getCaptureDistortion();

	int getWidth() { return captureWidth; }
	int getHeight() { return captureHeight; }
	bool getUndistort() { return captureUndistort; }
	void setUndistort(bool undistort) { captureUndistort = undistort; }
protected:
	videoInput *vi; int camIndex;
	CvMat* captureParams, *captureDistortion;
	bool loadCaptureParams(char *filename);
	int captureWidth, captureHeight;
	bool captureUndistort;
	CvMat* mDistortX, *mDistortY;
private:
	int principalX, principalY;
};

class Video: public Capture {
public:
	Video(char *videoFile);
	Video(char *videoFile, char* parametersFile);
};

class Camera: public Capture {
public:
	Camera() {init(0, cvSize(320,240), 0);}
	Camera(char* parametersFile) {init(0, cvSize(320,240), parametersFile);}
	Camera(int cameraIndex) {init(cameraIndex, cvSize(320,240), 0);};
	Camera(int cameraIndex, char* parametersFile) {init(cameraIndex, cvSize(320,240), parametersFile);};
	Camera(int cameraIndex, CvSize imgSize, char* parametersFile) {init(cameraIndex, imgSize, parametersFile);};
private:
	void init(int cameraIndex, CvSize imgSize, char* parametersFile);
};

#endif