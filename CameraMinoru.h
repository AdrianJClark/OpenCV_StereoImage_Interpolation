#include "captureLibrary.h"
#include "videoInput.h"
#include <string>

using namespace std;

class CameraMinoru:public Capture {
public:
	CameraMinoru(int index, CvSize resolution, string camera_params) {
		vi.setVerbose(false);
		vi.setupDevice(index, resolution.width, resolution.height);

		loadCalibPattern(camera_params);
		initUndistortMaps(vi.getWidth(index), vi.getHeight(index));

		captureWidth = vi.getWidth(index); captureHeight = vi.getHeight(index);
		camIndex = index;
		captureUndistort=true;
	}

	IplImage* getFrame() {
		IplImage* newFrame = cvCreateImage(cvSize(captureWidth, captureHeight), IPL_DEPTH_8U, 3);
		newFrame->imageData = (char*)vi.getPixels(camIndex, false, true);
		
		if (captureUndistort) {
			IplImage *unDistortedFrame = cvCreateImage(cvGetSize(newFrame), newFrame->depth, newFrame->nChannels);
			cvRemap( newFrame, unDistortedFrame, mDistortX, mDistortY);
			cvReleaseImage(&newFrame); newFrame = unDistortedFrame;
		}

		return newFrame;
	}

	~CameraMinoru() {
		vi.stopDevice(camIndex);
		cvReleaseMat(&captureParams); cvReleaseMat(&captureDistortion); cvReleaseMat(&camRectify); cvReleaseMat(&camProjection); cvReleaseMat(&camReprojection);
		cvReleaseMat(&mDistortX); cvReleaseMat(&mDistortY);
	}

	bool getAutoWhiteBalance() {
		long pMin, pMax, pStep, pVal, pFlags, pDef;
		vi.getVideoSettingFilter(camIndex, vi.propWhiteBalance, pMin, pMax, pStep, pVal, pFlags, pDef); 
		return (pFlags==1);
	}

	void setAutoWhiteBalance(bool whiteBal) {
		if (whiteBal)
			vi.setVideoSettingFilter(camIndex, vi.propWhiteBalance, 0, 1); 
		else
			vi.setVideoSettingFilter(camIndex, vi.propWhiteBalance, 0, -1); 
		
	}

	CvMat* getReprojection() {
		return camReprojection;
	}

private:
	videoInput vi;
	int camIndex;
	CvMat *camRectify, *camProjection, *camReprojection;
	CvMat *mDistortX, *mDistortY;

	void loadCalibPattern(string camera_params) {
		CvMemStorage* storage = cvCreateMemStorage();
		// reading intrinsic parameters
		CvFileStorage* fstorage = cvOpenFileStorage(camera_params.c_str(), storage, CV_STORAGE_READ);
		if(!fstorage)
		{
			printf("Failed to open file %s\n", camera_params.c_str());
			return;
		}

		captureParams = (CvMat*)cvReadByName(fstorage, NULL, "M");
		captureDistortion = (CvMat*)cvReadByName(fstorage, NULL, "D");
		camRectify = (CvMat*)cvReadByName(fstorage, NULL, "R");
		camProjection = (CvMat*)cvReadByName(fstorage, NULL, "P");
		camReprojection = (CvMat*)cvReadByName(fstorage, NULL, "Q");

		if(!captureParams || !captureDistortion || !camRectify || !camProjection || !camReprojection)
		{
			printf("Failed to read intrinsic parameters from %s\n", camera_params.c_str());
			return;
		}
	    
		cvReleaseFileStorage(&fstorage);
	} 

	void initUndistortMaps(int width, int height) {
		mDistortX = cvCreateMat(height, width, CV_32F);
		mDistortY = cvCreateMat(height, width, CV_32F);
		cvInitUndistortRectifyMap(captureParams, captureDistortion, camRectify, camProjection, mDistortX, mDistortY);
	}
};

/*	vi.getVideoSettingCamera(cameranumber, VI.propFocus, focusmin, focusmax, focusstep, focusval, pflags, pdef);
	//VI.getVideoSettingFilter(cameranumber, VI.propGain, gainmin, gainmax, gainstep, gainval, pflags, pdef);
	//VI.getVideoSettingCamera(cameranumber, VI.propExposure, exposuremin, exposuremax, exposurestep, exposureval, pflags, pdef);
	VI.getVideoSettingFilter(cameranumber, VI.propWhiteBalance, balancemin, balancemax, balancestep, balanceval, pflags, pdef);
	VI.setVideoSettingCamera(cameranumber, VI.propFocus, focusmin);//set the focus to its most distant, this also turns off automatic adjust
	VI.setVideoSettingFilter(cameranumber, VI.propWhiteBalance, balanceval);//turns off auto adjust
*/
