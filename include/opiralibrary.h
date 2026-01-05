#ifndef OPIRALIBRARY_H
#define OPIRALIBRARY_H

#include <vector>
#include "cv.h"

namespace OpiraLibrary {
	struct Marker {
		CvSize size;
		std::string name;
		IplImage* image;
	};

	struct MarkerTransform {
		Marker marker;
		int *viewPort;
		double *transMat, *projMat;
		int score, index;

		MarkerTransform(): viewPort(0), projMat(0), transMat(0), score(0), index(-1) {}
		
		void clear() {
			if (viewPort!=0) free(viewPort);
			if (projMat!=0) free(projMat);
			if (transMat!=0) free(transMat);
		}

	};


	struct PointMatches {
	public:
		CvPoint2D32f *featMarker, *featImage; CvMat* homography;
		int count; float *score;
		
		PointMatches(): featMarker(0), featImage(0), score(0), count(0) { homography = cvCreateMat(3,3,CV_32FC1); }

		void resize(int size) {
			clear();
			featMarker=(CvPoint2D32f*)malloc(size*sizeof(CvPoint2D32f));
			featImage=(CvPoint2D32f*)malloc(size*sizeof(CvPoint2D32f));
			score=(float*)malloc(size*sizeof(float));
			homography = cvCreateMat(3,3,CV_32FC1);
			count=size;
		} 
		
		void clear() {
			if (featMarker!=0) free(featMarker); featMarker=0;
			if (featImage!=0) free(featImage); featImage=0;
			if (score!=0) free(score); score=0;
			cvReleaseMat(&homography);
			count=0;

		}

		void clone(PointMatches src)
		{
			resize(src.count);
			cvCopy(src.homography, homography);
			memcpy(featImage, src.featImage, src.count*sizeof(CvPoint2D32f));
			memcpy(featMarker, src.featMarker, src.count*sizeof(CvPoint2D32f));
			memcpy(score, src.score, src.count*sizeof(float));
		}
	};


	class RegistrationAlgorithm {
	public:
		virtual std::vector<PointMatches> findAllMatches(IplImage* image);
		virtual PointMatches findMatches(IplImage* image, int index);
		virtual char* getName();
		virtual bool addMarker(Marker marker);
		virtual ~RegistrationAlgorithm();
	};


	struct Feature {
		CvPoint2D32f position; 
		float* descriptor;
		Feature(int descriptorLength) {descriptor = (float*)malloc(descriptorLength*sizeof(float));}
		Feature(int descriptorLength, float* descript) {descriptor = (float*)malloc(descriptorLength*sizeof(float)); memcpy(descriptor, descript, descriptorLength*sizeof(float));}
	};

	class FeaturePointVector: public std::vector<Feature> {
	public:
		int descriptorLength;
		void clear() {
			for (unsigned int i=0; i<size(); i++) free(at(i).descriptor);
		}
	};

	class RegistrationPointBasedAlgorithm: public RegistrationAlgorithm {
	public:
		~RegistrationPointBasedAlgorithm();
		std::vector<PointMatches> findAllMatches(IplImage* image);
		PointMatches findMatches(IplImage* image, int index);
		virtual char* getName();
		bool addMarker(Marker marker);
	protected:
		struct markerTree {cv::flann::Index* tree; cv::Mat *treePoints; CvPoint2D32f* points; int numPoints; };

		void constructANN(FeaturePointVector pa);
		PointMatches matchANN(FeaturePointVector pa, float thresh, markerTree mt);
		std::vector <markerTree> markerTrees;
		virtual FeaturePointVector performRegistration(IplImage* image);
	};


	class Registration {
	public:
		Registration();
		virtual ~Registration();
		virtual std::vector<MarkerTransform> performRegistration(IplImage* frame_input, CvMat* captureParams, CvMat* captureDistortion);
		Marker getMarker(int index);
		virtual void addMarker(char* markerName);
		static void calcViewpoint(CvMat* captureParams, CvMat* captureDistortion, CvSize imgSize, int *viewPort[4]);
		static void calcProjection(CvMat* captureParams, CvMat* captureDistortion, CvSize imgSize, double *projMat[16]);
		void calcTransform(CvMat *rotVector, CvMat *transVector, double *transMat[16]);
		void calcTransform(CvMat *homography, double *transMat[16]);
		CvMat* homographyToCameraExtrinsics(CvMat* H, CvMat* camParams);
	protected:
		std::vector<Marker> markers;
	};

	class RegistrationPointBased: public Registration {
	public:
		~RegistrationPointBased();
		RegistrationPointBased();
		bool displayImage;
	protected:
		static void displayMatches(IplImage *marker, IplImage *scene, PointMatches matches, const char* windowName, const char* text="",  float markerScale=1.0, float frameScale=1.0);
		static void processMatches(PointMatches m, CvMat* captureParams, CvMat* captureDistortion, CvMat** rotVector, CvMat** transVector);
		MarkerTransform computeMarkerTransform(PointMatches pMatch, int index, CvSize frameSize, CvMat *captureParams, CvMat *captureDistortion);
		MarkerTransform computeMarkerTransform(CvMat* homography, int matchCount, int index, CvSize frameSize, CvMat *captureParams, CvMat *captureDistortion);

		RegistrationAlgorithm *regAlgorithm;
		std::string windowName;
		int minRegistrationCount, minOptFlowCount;
	};



	class RegistrationPointBasedStandard:public RegistrationPointBased {
	public:
		RegistrationPointBasedStandard(char* markerFilename, RegistrationAlgorithm *registrationAlgorithm);
		RegistrationPointBasedStandard(std::vector<char*> markerFilenames, RegistrationAlgorithm *registrationAlgorithm);
		virtual void addMarker(char* markerFilename);
		~RegistrationPointBasedStandard();
		std::vector<MarkerTransform> performRegistration(IplImage* frame_input, CvMat* captureParams, CvMat* captureDistortion);
	protected:
		void destroyWindows();
	};

	class RegistrationPointBasedOpticalFlow:public RegistrationPointBasedStandard {
	public:
		RegistrationPointBasedOpticalFlow(char* markerFilename, RegistrationAlgorithm *registrationAlgorithm);
		RegistrationPointBasedOpticalFlow(std::vector<char*> markerFilenames, RegistrationAlgorithm *registrationAlgorithm);
		virtual void addMarker(char* markerFilename);
		~RegistrationPointBasedOpticalFlow();
		std::vector<MarkerTransform> performRegistration(IplImage* frame_input, CvMat* captureParams, CvMat* captureDistortion);
	protected:
		PointMatches opticalFlow(IplImage *previousImage, PointMatches prevMatches, IplImage *currentImage);
		IplImage* previousImage;
		std::vector<PointMatches> previousMatches;
	};

	class RegistrationPointBasedOpira:public RegistrationPointBasedOpticalFlow {
	public:
		RegistrationPointBasedOpira(char* markerFilename, RegistrationAlgorithm *registrationAlgorithm);
		RegistrationPointBasedOpira(std::vector<char*> markerFilenames, RegistrationAlgorithm *registrationAlgorithm);
		~RegistrationPointBasedOpira();
		std::vector<MarkerTransform> performRegistration(IplImage* frame_input, CvMat* captureParams, CvMat* captureDistortion);
	protected:
		void destroyWindows();
		PointMatches undistortRegister(IplImage* frame_input, int index, CvMat* homography);
	};

	PointMatches Ransac(PointMatches corspMap);
};


#endif
