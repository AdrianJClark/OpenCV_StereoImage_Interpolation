#include "cv.h"
#include "cvaux.h"
#include "highgui.h"
#include "CameraMinoru.h"
#include "RANSAC.h"
#include "opiralibrary.h"
#include "pointbasedocvsurfalgorithm.h"

using namespace OpiraLibrary;

struct markerTree {cv::flann::Index* tree; cv::Mat *treePoints; CvPoint2D32f* points; int numPoints; };

void cvShowStereoImage(char *windowTitle, IplImage *frameL, IplImage *frameR);
void constructANN(FeaturePointVector features);
PointMatches matchANN(FeaturePointVector features, float thresh);
markerTree mt; 
void Do_Morphing(IplImage* RightImage, IplImage* LeftImage, CvMatrix3 *F_Matrix);
void myMorph(IplImage* imgLeft, IplImage* imgRight, CvMatrix3 fundMat);

float position=0.0;

void main() {
	Capture *camLeft = new Camera(1, cvSize(320,240), "leftCamera2.yml");
	Capture *camRight = new Camera(0, cvSize(320,240), "rightCamera2.yml");

	bool doLoop=true;
	while (doLoop) {
		IplImage* imgLeft = camLeft->getFrame();
		IplImage* imgRight = camRight->getFrame();

		cvShowStereoImage("input", imgLeft, imgRight);

		RegistrationPointBasedOCVSurf *surf = new RegistrationPointBasedOCVSurf();

		FeaturePointVector pLeft = surf->performRegistration(imgLeft);
		constructANN(pLeft);

		FeaturePointVector pRight = surf->performRegistration(imgRight);
		PointMatches m = matchANN(pRight, 1.3);

		IplImage *dualImage = cvCreateImage(cvSize(imgLeft->width + imgRight->width, imgLeft->height<imgRight->height?imgRight->height:imgLeft->height), imgLeft->depth, imgLeft->nChannels);
		cvSetImageROI(dualImage, cvRect(0,0,imgLeft->width, imgLeft->height)); cvCopy(imgLeft, dualImage);
		cvSetImageROI(dualImage, cvRect(imgLeft->width,0,imgRight->width, imgRight->height)); cvCopy(imgRight, dualImage);
		cvResetImageROI(dualImage);

		PointMatches r = Ransac(m); 
		for (int i=0; i<r.count; i++) {
			cvLine(dualImage, cvPoint(r.featMarker[i].x, r.featMarker[i].y), cvPoint(imgLeft->width + r.featImage[i].x, r.featImage[i].y), cvScalar(0,255,255));
		}

		cvNamedWindow("matches"), cvShowImage("matches", dualImage);
		cvReleaseImage(&dualImage);

		int *pL = (int*) malloc(sizeof(int)*r.count*2); int *pR = (int*) malloc(sizeof(int)*r.count*2);
		float *f = (float*)malloc(sizeof(float)*9);
		CvMatrix3 fundMat;
		for (int i=0; i<r.count; i++) {
			pL[i*2] = r.featMarker[i].x; pL[i*2+1] = r.featMarker[i].y;
			pR[i*2] = r.featImage[i].x; pR[i*2+1] = r.featImage[i].y;
		}

		/*1. Find the fundamental matrix using the correspondence points in the two
			images of cameras by calling the function FindFundamentalMatrix. */
		cvFindFundamentalMatrix(pL, pR, r.count, 0, f);
		fundMat.m[0][0] = f[0]; fundMat.m[0][1] = f[1]; fundMat.m[0][2] = f[2]; 
		fundMat.m[1][0] = f[3]; fundMat.m[1][1] = f[4]; fundMat.m[1][2] = f[5]; 
		fundMat.m[2][0] = f[6]; fundMat.m[2][1] = f[7]; fundMat.m[2][2] = f[8]; 

		//(float*)fundMat.m);

		myMorph(imgRight, imgLeft, fundMat);
		//Do_Morphing(imgLeft, imgRight, &fundMat);

		free(pL); free(pR);
		m.clear();	r.clear();

		switch (cvWaitKey(1)) {
			case 27: doLoop=false; break;
				break;
			case 'a': position-=0.1; break;
			case 'd': position+=0.1; break;
		}

		pLeft.clear(); pRight.clear();
		delete surf;
		
		cvReleaseImage(&imgLeft);
		cvReleaseImage(&imgRight);

	}
	delete camLeft;
	delete camRight;

}
	/*		IplImage* velx = cvCreateImage(cvGetSize(imgLeft), IPL_DEPTH_32F, 1);
		IplImage* vely = cvCreateImage(cvGetSize(imgLeft), IPL_DEPTH_32F, 1);

		IplImage* imgLBW = cvCreateImage(cvGetSize(imgLeft), IPL_DEPTH_8U, 1); cvConvertImage(imgLeft, imgLBW);
		IplImage* imgRBW = cvCreateImage(cvGetSize(imgRight), IPL_DEPTH_8U, 1); cvConvertImage(imgRight, imgRBW);

		cvCalcOpticalFlowLK(imgLBW, imgRBW, cvSize(3,3), velx, vely);

//		cvNamedWindow("velx"); cvShowImage("velx", velx);
//		cvNamedWindow("vely"); cvShowImage("vely", velx);

		cvReleaseImage(&imgLBW); cvReleaseImage(&imgRBW); 
		cvReleaseImage(&velx); cvReleaseImage(&vely); 

				int lResult = cvFindChessboardCorners(imgLeft, cvSize(8, 6), &lCorners[0], &lCornerCount);
				int rResult = cvFindChessboardCorners(imgRight, cvSize(8, 6), &rCorners[0], &rCornerCount);
				cvDrawChessboardCorners(imgLeft, cvSize(8,6), &lCorners[0], lCornerCount, lResult);
				cvDrawChessboardCorners(imgRight, cvSize(8,6), &rCorners[0], rCornerCount, rResult);
				cvShowStereoImage("chessboard", imgLeft, imgRight);
			
				IplImage *imgLeftWarp = cvCreateImage(cvGetSize(imgLeft), IPL_DEPTH_8U, 3); //cv
				cvWarpPerspective(imgLeft, imgLeftWarp, homo,1+8, cvScalar(255,0,255));
				//cvSet(imgLeftWarp, cvScalar(255,0,255,0)); //cvFillImage(imgLeft, 0);
				cvNamedWindow("Burp"); cvShowImage("Burp", imgLeftWarp); 

				IplImage *imgBlend = cvCreateImage(cvGetSize(imgRight), IPL_DEPTH_8U, 3);
				for (int y=0; y<imgBlend->height; y++) {
					for (int x=0; x<imgBlend->width*3; x+=3) {
						unsigned char x1, y1, z1, x2, y2, z2, x3, y3, z3;
						x1 = imgLeftWarp->imageData[x+y*imgBlend->width*3];
						y1 = imgLeftWarp->imageData[x+y*imgBlend->width*3+1];
						z1 = imgLeftWarp->imageData[x+y*imgBlend->width*3+2];

						x2 = imgRight->imageData[x+y*imgBlend->width*3];
						y2 = imgRight->imageData[x+y*imgBlend->width*3+1];
						z2 = imgRight->imageData[x+y*imgBlend->width*3+2];

						if (x1!=255 && y1!=0 && z1!=255) {
							x3 = (unsigned char)(x1*0.5+x2*0.5); y3 = (unsigned char)(y1*0.5+y2*0.5); z3 = (unsigned char)(z1*0.5+z2*0.5);
							imgBlend->imageData[x+y*imgBlend->width*3] = x3;
							imgBlend->imageData[x+y*imgBlend->width*3+1] = y3;
							imgBlend->imageData[x+y*imgBlend->width*3+2] = z3;
						} else {
							imgBlend->imageData[x+y*imgBlend->width*3] = x2;
							imgBlend->imageData[x+y*imgBlend->width*3+1] = y2;
							imgBlend->imageData[x+y*imgBlend->width*3+2] = z2;
						}

					}
				}

				cvNamedWindow("bleh"); cvShowImage("bleh", imgBlend);
		cvReleaseImage(&imgLeftWarp); cvReleaseImage(&imgBlend);

		switch (cvWaitKey(1)) {
			case 27: doLoop=false; break;
				break;
			case ' ':
				if (lResult && rResult) {
					CvPoint2D32f pointLeft[4];
					pointLeft[0] = lCorners[0];	pointLeft[1] = lCorners[7];	pointLeft[2] = lCorners[lCornerCount-1]; pointLeft[3] = lCorners[lCornerCount-8];

					CvPoint2D32f pointRight[4];
					pointRight[0] = rCorners[0]; pointRight[1] = rCorners[7]; pointRight[2] = rCorners[rCornerCount-1];	pointRight[3] = rCorners[rCornerCount-8];

					cvGetPerspectiveTransform(pointLeft, pointRight, homo);


				}

		}

		cvReleaseImage(&imgLeft);
		cvReleaseImage(&imgRight);

	}
cvReleaseMat(&homo);
	delete camLeft;
	delete camRight;

}
*/

void constructANN(FeaturePointVector features) {
	//Create and initialise a new marker tree object
	mt.numPoints = features.size();
	mt.points = (CvPoint2D32f*)malloc(sizeof(CvPoint2D32f)*features.size());
	mt.treePoints = new cv::Mat(features.size(), features.descriptorLength, CV_32F);

    //Populate the tree points and point positions
	for(unsigned int i = 0; i < features.size(); i++ )
    {
		memcpy(mt.treePoints->ptr<float*>(i), features.at(i).descriptor, features.descriptorLength*sizeof(float));
		mt.points[i] = features.at(i).position; 
    }

	//Initialise the tree
	mt.tree = new cv::flann::Index(*(mt.treePoints), cv::flann::KDTreeIndexParams(4));  // using 4 randomized kdtrees
}

PointMatches matchANN(FeaturePointVector features, float thresh) {
	//Initialise matrices for the features, indices and distances
	cv::Mat m_features(features.size(), features.descriptorLength, CV_32F);
	cv::Mat m_indices(features.size(), 2, CV_32S);
    cv::Mat m_dists(features.size(), 2, CV_32F);

	//Create and initialise the returning point match object
	PointMatches matches;

	//If there's no features, just return an empty structure
	if(features.size()==0) return matches;

	//Initialise feature search set
    for(unsigned int i = 0; i < features.size(); i++ )
    {
		memcpy(m_features.ptr<float*>(i), features.at(i).descriptor, features.descriptorLength*sizeof(float));
    }

	//Perform search
	mt.tree->knnSearch(m_features, m_indices, m_dists, 2, cv::flann::SearchParams(features.descriptorLength)); // maximum number of leafs checked

	//Search through the results. Remember every match where the best match * threshold is less than the second best match
	int *tmp1 = new int[features.size()];
	for (unsigned int i=0; i<features.size(); i++) 
		if (m_dists.at<float>(i,0)*thresh <=m_dists.at<float>(i,1)) { tmp1[matches.count] = i; matches.count++; }

	//Initialise the returning point match object
	matches.resize(matches.count);

	//Loop through and clean up
	for (int i=0; i<matches.count; i++) {
		matches.featMarker[i] =  mt.points[m_indices.at<int>(tmp1[i],0)];
		matches.featImage[i] = features.at(tmp1[i]).position;
		matches.score[i] = m_dists.at<float>(tmp1[i],0);
	}

	//Clean up
	free(tmp1);
	m_features.release(); m_indices.release(); m_dists.release();

	return matches;
}
void cvShowStereoImage(char *windowTitle, IplImage *frameL, IplImage *frameR) {
	IplImage *dualImage = cvCreateImage(cvSize(frameL->width + frameR->width, frameL->height<frameR->height?frameR->height:frameL->height), frameL->depth, frameL->nChannels);
	cvSetImageROI(dualImage, cvRect(0,0,frameL->width, frameL->height)); cvCopy(frameL, dualImage);
	cvSetImageROI(dualImage, cvRect(frameL->width,0,frameR->width, frameR->height)); cvCopy(frameR, dualImage);
	cvResetImageROI(dualImage);

	cvNamedWindow(windowTitle); cvShowImage(windowTitle, dualImage);
	cvReleaseImage(&dualImage);
}








void Do_Morphing(IplImage* RightImage, IplImage* LeftImage, CvMatrix3 *F_Matrix)
 {
 	CvSize ImgSize;
 	ImgSize.width=RightImage->width*3;
 	ImgSize.height=RightImage->height*3;
 	int line_count;
 	cvMakeScanlines(0,ImgSize,0,0,0,0,&line_count);
 	//line_count=2400;
 	int* scanlines1=(int*)(calloc( line_count * 2, sizeof(int) * 4));
 	int* scanlines2=(int*)(calloc( line_count * 2, sizeof(int) * 4));
 	int* scanlinesA=(int*)(calloc( line_count * 2, sizeof(int) * 4));
 	int* lenghts1=(int*)(calloc( line_count * 2, sizeof(int)*4));
 	int* lenghts2=(int*)(calloc( line_count * 2, sizeof(int)*4));
 	int* lenghts=(int*)(calloc( line_count * 2, sizeof(int)*4));
 	uchar* dst1=(uchar*)(malloc(ImgSize.width * (ImgSize.height+1) * 3 * sizeof(uchar)));
 	uchar* dst2=(uchar*)(malloc(ImgSize.width * (ImgSize.height+1) * 3 * sizeof(uchar)));
 	uchar* dst_pix=(uchar*)(calloc(ImgSize.width * (ImgSize.height+1), 3 * sizeof(uchar)));
 	int* runs1=(int*)(calloc(ImgSize.width * ImgSize.height * 2, 2 * sizeof(int)));
 	int* runs2=(int*)(calloc(ImgSize.width * ImgSize.height * 2, 2 * sizeof(int)));;
 	int* first_corr=(int*)(calloc(ImgSize.width * ImgSize.height * 2, 2 *  sizeof(int)));
 	int* second_corr=(int*)(calloc(ImgSize.width * ImgSize.height * 2, 2 * sizeof(int)));;
 	int* num_runs1=(int*)(calloc(ImgSize.width * ImgSize.height * 2, 2 *  sizeof(int)));;
 	int* num_runs2=(int*)(calloc(ImgSize.width * ImgSize.height * 2, 2 *  sizeof(int)));;
 
 	ImgSize.width=RightImage->width;
 	ImgSize.height=RightImage->height;
 
 	cvMakeScanlines(F_Matrix,ImgSize,scanlines1,scanlines2,lenghts1,lenghts2,&line_count);
 	
 	IplImage* image1=cvCloneImage(RightImage);
 	cvPreWarpImage(line_count,image1,dst1,lenghts1,scanlines1);
 	
 	IplImage* image2=cvCloneImage(LeftImage);
 	cvPreWarpImage(line_count,image2,dst2,lenghts2,scanlines2);
 		
 	cvFindRuns(line_count,dst1,dst2,lenghts1,lenghts2,runs1,runs2,num_runs1,num_runs2);
 
 	cvDynamicCorrespondMulti(line_count,runs1,num_runs1,runs2,num_runs2,first_corr,second_corr);
 	
 	float alpha=(float(0)/100.f);
 
 	cvMakeAlphaScanlines(scanlines1,scanlines2,scanlinesA,lenghts,line_count,alpha);
 	
 	cvMorphEpilinesMulti(line_count,dst1,lenghts1,dst2,lenghts2,dst_pix,lenghts,alpha,runs1,num_runs1,runs2,num_runs2,first_corr,second_corr);
 
 	IplImage* image=cvCreateImage(cvGetSize(RightImage),8,3);
 	
 	cvPostWarpImage(line_count,dst_pix,lenghts,image,scanlinesA);
 	
  	cvShowImage("Eye",image);
 
 	cvShowImage("Big Right Eye",image1);
 	cvShowImage("Big Left Eye",image2);
 
 
     if (scanlines1 != 0)   free (scanlines1);
     if (scanlines2 != 0)   free (scanlines2);
     if (scanlinesA != 0)   free (scanlinesA);
     if (lenghts1 != 0)   free (lenghts1);
     if (lenghts2 != 0)   free (lenghts2);
     if (lenghts  != 0)   free (lenghts);
     if (dst1 != 0)   free (dst1);
     if (dst2 != 0)   free (dst2);
     if (dst_pix != 0)   free (dst_pix);
     if (runs1 != 0)   free (runs1);
     if (runs2 != 0)   free (runs2);
     if (first_corr != 0)   free (first_corr);
     if (second_corr != 0)   free (second_corr);
     if (num_runs1 != 0)   free (num_runs1);
     if (num_runs2 != 0)   free (num_runs2);
 }

 void myMorph(IplImage* imgLeft, IplImage* imgRight, CvMatrix3 fundMat) {
		/*2. Find the number of scanlines in the images for the given fundamental matrix
		by calling the function FindFundamentalMatrix with null pointers to the
		scanlines.*/
		int scanLineCount;
		cvMakeScanlines(&fundMat, cvSize(320,240), 0,0, 0, 0, &scanLineCount);
		
		/*3. Allocate enough memory for:
		— scanlines in the first image, scanlines in the second image, scanlines in the
		virtual image (for each numscan*2*4*sizeof(int));*/
		int *scanLeft = (int*)malloc(sizeof(int)*2*4*scanLineCount);
		int *scanRight = (int*)malloc(sizeof(int)*2*4*scanLineCount);
		int *scanVirtual = (int*)malloc(sizeof(int)*2*4*scanLineCount);

		/*— lengths of scanlines in the first image, lengths of scanlines in the second
		image, lengths of scanlines in the virtual image (for each
		numscan*2*4*sizeof(int));*/
		int *scanLengthLeft = (int*)malloc(sizeof(int)*2*4*scanLineCount);
		int *scanLengthRight = (int*)malloc(sizeof(int)*2*4*scanLineCount);
		int *scanLengthVirtual = (int*)malloc(sizeof(int)*2*4*scanLineCount);

		/*— buffer for the prewarp first image, the second image, the virtual image (for
		each width*height*2*sizeof(int));*/
//		uchar *bufferLeft = (uchar*)malloc(sizeof(uchar)*max(imgLeft->width,imgLeft->height)*scanLineCount*3);
//		uchar *bufferRight = (uchar*)malloc(sizeof(uchar)*max(imgLeft->width,imgLeft->height)*scanLineCount*3);
//		uchar *bufferVirtual = (uchar*)malloc(sizeof(uchar)*max(imgLeft->width,imgLeft->height)*scanLineCount*3);
 		uchar* bufferLeft=(uchar*)(malloc(imgLeft->width *3* (imgLeft->height+1) * 3 * 3 * sizeof(uchar)));
 		uchar* bufferRight=(uchar*)(malloc(imgLeft->width *3* (imgLeft->height+1) * 3 * 3 * sizeof(uchar)));
 		uchar* bufferVirtual=(uchar*)(malloc(imgLeft->width *3* (imgLeft->height+1) * 3* 3 * sizeof(uchar)));


		/*	— data runs for the first image and the second image (for each
		width*height*4*sizeof(int));*/
		int *dataRunLeft = (int*)malloc(sizeof(int)*imgLeft->width*imgLeft->height*3*4);
		int *dataRunRight = (int*)malloc(sizeof(int)*imgLeft->width*imgLeft->height*3*4);

		/*NOT INCLUDED -numbers of runs in each scanline for the first and second images (for each
		width*height*4*sizeof(int)*/
	//	int *numRunsLeft = (int*)malloc(sizeof(int)*scanLineCount*2*4);
	//	int *numRunsRight = (int*)malloc(sizeof(int)*scanLineCount*2*4);

		/*— correspondence data for the first image and the second image (for each
		width*height*2*sizeof(int));*/
		//int *correspLeft = (int*)malloc(sizeof(int)*max(imgLeft->width,imgLeft->height)*scanLineCount*3);
		//int *correspRight = (int*)malloc(sizeof(int)*max(imgLeft->width,imgLeft->height)*scanLineCount*3);
	 	int* correspLeft=(int*)malloc(imgLeft->width * imgLeft->height *3* 2* 2 *  sizeof(int));
 		int* correspRight=(int*)malloc(imgLeft->width * imgLeft->height *3* 2* 2 * sizeof(int));

		/*-numbers of lines for the first and second images (for each
		width*height*4*sizeof(int)*/
		int *numRunsLeft = (int*)malloc(sizeof(int)*imgLeft->width*imgLeft->height*3*4);
		int *numRunsRight = (int*)malloc(sizeof(int)*imgLeft->width*imgLeft->height*3*4);

		/*4. Find scanlines coordinates by calling the function FindFundamentalMatrix.*/
		cvMakeScanlines(&fundMat, cvSize(320,240), scanLeft, scanRight, scanLengthLeft, scanLengthRight, &scanLineCount);
	
		/*5. Prewarp the first and second images using scanlines data by calling the
		function PreWarpImage.*/
		cvPreWarpImage(scanLineCount, imgLeft, bufferLeft, scanLengthLeft, scanLeft);
		cvPreWarpImage(scanLineCount, imgRight, bufferRight, scanLengthRight, scanRight);

		/*6. Find runs on the first and second images scanlines by calling the function
		FindRuns.*/
		cvFindRuns(scanLineCount, bufferLeft, bufferRight, scanLengthLeft, scanLengthRight, dataRunLeft, dataRunRight, numRunsLeft, numRunsRight);

		/*7. Find correspondence information by calling the function
		DynamicCorrespondMulti.*/
		cvDynamicCorrespondMulti(scanLineCount, dataRunLeft, numRunsLeft, dataRunRight, numRunsRight, correspLeft, correspRight);

		/*8. Find coordinates of scanlines in the virtual image for the virtual camera
		position alpha by calling the function MakeAlphaScanlines.*/
		cvMakeAlphaScanlines(scanLeft, scanRight, scanVirtual, scanLengthVirtual, scanLineCount, position);
		
		/*9. Morph the prewarp virtual image from the first and second images using
		correspondence information by calling the function MorphEpilinesMulti.*/
		cvMorphEpilinesMulti(scanLineCount, bufferLeft, scanLengthLeft, bufferRight, scanLengthRight, bufferVirtual, scanLengthVirtual, position, dataRunLeft, numRunsLeft, dataRunRight, numRunsRight, correspLeft, correspRight);

		IplImage* imgOut = cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 3);
		/*10. Postwarp the virtual image by calling the function PostWarpImage.*/
		cvPostWarpImage(scanLineCount, bufferVirtual, scanLengthVirtual, imgOut, scanVirtual);

		/*11. Delete moire from the resulting virtual image by calling the function
		DeleteMoire.*/
		cvDeleteMoire(imgOut);


		cvNamedWindow("out"); cvShowImage("out", imgOut);
		cvReleaseImage(&imgOut);


		free(scanLeft); free(scanRight); free(scanVirtual);
		free(scanLengthLeft); free(scanLengthRight); free(scanLengthVirtual);
		free(bufferLeft); free(bufferRight); free(bufferVirtual);
		free(dataRunLeft); free(dataRunRight); 
		free(correspLeft); free(correspRight); 

}