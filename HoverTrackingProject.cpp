//============================================================================
// Name        : HoverTrackingProject.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
// ArasProject.cpp : Defines the entry point for the console application.
//


//openCV include files.
#include "highgui.h"
#include "cv.h"
#include "cxcore.h"
#include "ml.h"
//#include "opencv.hpp"

#include "FlyCapture2.h"

//TUIO files
//#include "TuioListener.h"
//#include "TuioClient.h"
//#include "TuioObject.h"
//#include "TuioCursor.h"
//#include "TuioPoint.h"

//#include <list>
#include <math.h>

#include "TuioServer.h"
#include "TuioClient.h"
#include "TuioCursor.h"
//#include <SDL.h>
//#include <SDL_thread.h>

using namespace FlyCapture2;
using namespace TUIO;
using namespace osc;

void playVideo();
void displayOpticalFlow(IplImage *previousFrame, IplImage *currentFrame);
void drawArrows(IplImage *currentFrame, IplImage *velx, IplImage *vely, CvSize winSize);
void drawAverage(IplImage *currentFrame, IplImage *velx, IplImage *vely, CvSize winSize);
void findTopPoint(IplImage *image, IplImage *velx, IplImage *vely, CvSize winSize);
void findTopOfFinger(IplImage *image);
void getBackground(CvCapture *capture);
void removeBackground(IplImage *image);
void setBlack(IplImage *image, int i, int j);
bool isBlack(IplImage *image,int i, int j);
bool isBlackLine(IplImage *image, CvPoint leftPoint, CvPoint rightPoint);
CvPoint getAverageLoc(IplImage *image);
void mouseDown(int Event, int x, int y, int flags, void* param);
CvScalar getColour(IplImage *image,int i, int j);
void displayResults(IplImage *image);
IplImage *grabImage();
IplImage *convertImageToOpenCV(Image* pImage);
void initiateCamera();
void sendPoint(CvPoint point,IplImage *image, int state);
void cropImage(IplImage *image);
bool fingerPressed(IplImage *image, CvPoint fingerTip);
void createCalibrationMatrix();
IplImage *calibrateImage(IplImage *image);

//The labels min,max, and average are used by the math library.  Had to name them something different.
IplImage *aveX;
IplImage *stdDev;
CvPoint mouseLocation;
//IplImage *minX,*maxX;

Camera Dfly;
PGRGuid gui;

TuioServer *tuioServer;
TuioCursor *cursor;
TuioObject *hover;

IplImage *mapx;
IplImage *mapy;
//bool firstCursor = true;

//int _tmain(int argc, _TCHAR* argv[])
int main(int argc, char* argv[])
{
	//int port = 3333;
	//const char *host = "localhost";
	tuioServer = new TuioServer();

	CvMat *intrinsic = NULL;
	CvMat *distortion = NULL;
	intrinsic = (CvMat*) cvLoad("Intrinsics.xml");
	distortion = (CvMat*) cvLoad("Distortions.xml");
	mapx = cvCreateImage(cvSize(640,480),IPL_DEPTH_32F,1);
	mapy = cvCreateImage(cvSize(640,480),IPL_DEPTH_32F,1);
	cvInitUndistortMap(intrinsic,distortion,mapx,mapy);
	if(!intrinsic || !distortion){
		//cout << "Failed to load calibration matrices." << endl;
		printf("Failed to load calibration matrices.\n");
		return(0);
	}

	playVideo();
	//createCalibrationMatrix();

	//delete tuioServer;
	//printf("Done, pausing.\n");
	//system("pause");
	return 0;
}
void playVideo(){

	bool saveVideo = false;
	bool displayVideo = true;
	bool subtractBackground = false;
	bool pointgreyCamera = false;
	bool opticalFlow = false;
	//bool remBack = true;

	const char *videoFilename;
	CvCapture *video = NULL;
	CvSize videoSize;
	double fps = 20;
	int totalFrames = 0;
	IplImage *previousFrame = NULL;
	IplImage *frameCopy = NULL;

	if(!pointgreyCamera){
		videoFilename = "touch_to_left_hover_to_right.avi";
		video = cvCreateFileCapture(videoFilename);
		if(!video){
			printf("Cannot find video file.\n");
			return;
			//system("pause");
		}
		if(subtractBackground){
			getBackground(video);
		}
		cvSetCaptureProperty(video,CV_CAP_PROP_POS_FRAMES,0);
		videoSize  = cvSize((int) cvGetCaptureProperty(video,CV_CAP_PROP_FRAME_WIDTH),(int) cvGetCaptureProperty(video,CV_CAP_PROP_FRAME_HEIGHT));
		fps = cvGetCaptureProperty(video,CV_CAP_PROP_FPS);
		totalFrames = (int) cvGetCaptureProperty(video,CV_CAP_PROP_FRAME_COUNT);
	}
	else{
		initiateCamera();
	}

	CvVideoWriter *writer;
	if(saveVideo){
		writer = cvCreateVideoWriter("rawOutput.avi",-1,fps,videoSize);
		if(!writer){
			printf("Failed to initiate video writer, fps is %f, video size is %d,%d",fps,videoSize.width,videoSize.height);
		}
	}
	const char *windowName = "pointer";
	if(displayVideo){
		if(cvNamedWindow(windowName,CV_WINDOW_AUTOSIZE)){
			//Failed to create window for viewing the image.
		}
	}

	mouseLocation = cvPoint(0,0);

	int key;
	//IplImage *videoFrame = NULL;
	IplImage *pauseCopy = NULL;
	IplImage *pCopy = NULL;
	int lastKey = 'f';
	//IplImage *bwFrame;

	if(!video && !pointgreyCamera){
		//system("pause");
	}

	for(int frame = 0; frame < totalFrames || pointgreyCamera; frame++){
		//printf("Frame %d \n",frame);
		IplImage *videoFrame = NULL;
		IplImage *image = NULL;

		if(pointgreyCamera){
			image = grabImage();
			//cropImage(videoFrame);
			//cvCanny(videoFrame,videoFrame,30,120,3);
		}
		else{
			image = cvQueryFrame(video);
			//cropImage(videoFrame);
		}
		videoFrame = calibrateImage(image);

		if(!videoFrame){
			printf("videoFrame is null.\n");
			break;
		}

		//cvSaveImage("testing.bmp",videoFrame);
		cvSetMouseCallback(windowName,mouseDown, (void*) videoFrame);

		if(subtractBackground){
			//removeBackground(videoFrame);
		}
		if(opticalFlow){
			cvSmooth(videoFrame,videoFrame,CV_MEDIAN);
			cvThreshold(videoFrame,videoFrame,20,0,CV_THRESH_TOZERO);
		}
		//cvSmooth(videoFrame,videoFrame,CV_BLUR,5);

		if(frame > 0){
			cvCopyImage(videoFrame,frameCopy);
			if(lastKey == 'f'){
				if(opticalFlow){
					displayOpticalFlow(previousFrame,videoFrame);
				}
				else{
					findTopOfFinger(videoFrame);
				}
			}
			cvCopyImage(frameCopy,previousFrame);
		}
		else{
			frameCopy = cvCloneImage(videoFrame);
			previousFrame = cvCloneImage(videoFrame);
			//cvSaveImage("testing.bmp",videoFrame);
		}

		//cvSaveImage("frame.bmp",videoFrame);
		if(displayVideo){
			key = cvWaitKey((int) (200/fps));
			if(key == 'p'){
				//Pauses the video.
				key = -1;
				if(!pauseCopy){
					pauseCopy = cvCloneImage(videoFrame);
				}
				else{
					cvCopyImage(videoFrame,pauseCopy);
				}
				if(!pCopy){
					pCopy = cvCloneImage(videoFrame);
				}
				else{
					cvCopyImage(videoFrame,pCopy);
				}
				while(true){
					key = cvWaitKey(10);
					cvCopyImage(pCopy,pauseCopy);
					displayResults(pauseCopy);
					cvShowImage(windowName,pauseCopy);
					if(key == 'p'){
						break;
					}
					if(key == 's'){
						cvSaveImage("opticalFlow.bmp",pCopy);
						cvSaveImage("originalImage.bmp",frameCopy);
					}
				}
			}
			if(key == 'q'){
				break;
			}
			if(key == 'o'){
				lastKey = key;
			}
			if(key == 'f'){
				lastKey = key;
			}
			if(key == 'b'){
				//remBack = !remBack;
				subtractBackground = !subtractBackground;
			}

			displayResults(videoFrame);
			cvShowImage(windowName,videoFrame);
		}

		if(saveVideo){
			cvWriteFrame(writer,videoFrame);
		}
		cvReleaseImage(&videoFrame);
		cvReleaseImage(&frameCopy);
	}
	if(saveVideo){
		cvReleaseVideoWriter(&writer);
	}
	if(displayVideo){
		cvDestroyWindow(windowName);
	}

	if(pauseCopy){
		cvReleaseImage(&pauseCopy);
	}

	//flycaptureStop(Dfly);
	Dfly.StopCapture();

	if(!pointgreyCamera){
		cvReleaseCapture(&video);
	}
	if(frameCopy){
		cvReleaseImage(&frameCopy);
	}
	//cvReleaseImage(&bwFrame);
	if(previousFrame){
		cvReleaseImage(&previousFrame);
	}
}
void displayOpticalFlow(IplImage *previousFrame, IplImage *currentFrame){

	CvSize winSize = cvSize(16,16);//Make sure this divides evenly with the video image
	CvSize shiftSize = cvSize(16,16);
	CvSize flowChartSize = cvSize((previousFrame->width - winSize.width)/shiftSize.width,(previousFrame->height - winSize.height)/shiftSize.height);
	//CvSize flowChartSize = cvGetSize(previousFrame);

	IplImage *velx = cvCreateImage(flowChartSize,IPL_DEPTH_32F,1);
	IplImage *vely = cvCreateImage(flowChartSize,IPL_DEPTH_32F,1);

	IplImage *bwPreviousFrame = cvCreateImage(cvGetSize(previousFrame),IPL_DEPTH_8U,1);
	IplImage *bwCurrentFrame = cvCreateImage(cvGetSize(previousFrame),IPL_DEPTH_8U,1);
	bwCurrentFrame->origin = IPL_ORIGIN_BL;
	bwPreviousFrame->origin = IPL_ORIGIN_BL;
	if(previousFrame->nChannels == 3){
		cvCvtColor(previousFrame,bwPreviousFrame,CV_BGR2GRAY);
		cvCvtColor(currentFrame,bwCurrentFrame,CV_BGR2GRAY);
	}
	else{
		cvCopyImage(previousFrame,bwPreviousFrame);
		cvCopyImage(currentFrame,bwCurrentFrame);
	}

	cvSmooth(bwPreviousFrame,bwPreviousFrame,CV_GAUSSIAN,5);
	cvSmooth(bwCurrentFrame,bwCurrentFrame,CV_GAUSSIAN,5);

	bwCurrentFrame->origin = IPL_ORIGIN_BL;
	bwPreviousFrame->origin = IPL_ORIGIN_BL;

	//Must convert to 8 bit greyscale first.

	//cvCalcOpticalFlowLK(bwPreviousFrame,bwCurrentFrame,winSize,velx,vely);
	//cvCalcOpticalFlowBM(bwPreviousFrame,bwCurrentFrame,winSize,cvSize(2,2),cvSize(16,16),0,velx,vely);
	cvCalcOpticalFlowBM(bwPreviousFrame,bwCurrentFrame,winSize,shiftSize,cvSize(16,16),0,velx,vely);
	drawArrows(currentFrame,velx,vely,winSize);
	//drawAverage(currentFrame,velx,vely,winSize);
	findTopPoint(currentFrame,velx,vely,winSize);

	cvReleaseImage(&velx);
	cvReleaseImage(&vely);
	cvReleaseImage(&bwPreviousFrame);
	cvReleaseImage(&bwCurrentFrame);
}
void drawArrows(IplImage *currentFrame, IplImage *velx, IplImage *vely, CvSize winSize){

	//Assumes width and height are the same.
	int arrowLength = winSize.width/2;
	float *velxD = (float*) velx->imageData;
	float *velyD = (float*) vely->imageData;
	CvPoint lineOrigin;
	CvPoint lineEnd;
	CvPoint boxCenter;
	CvPoint arrowHeadEnd;
	double pi = 3.141592654;
	double alpha = 45*(180/pi),theta, phi;
	int r = 5;
	int index;
	double scaleFactor = 0.2;
	double minVel = 16;
	double maxVel = 64;
	double maxFoundVel = 48;

	CvScalar colour = cvScalar(255,0,255);

	for(int i = 0; i < velx->width; i++){
		for(int j = 0; j < velx->height; j++){
			boxCenter.x = i*winSize.width + winSize.width/2;
			boxCenter.y = j*winSize.height + winSize.height/2;
			index = j*velx->width + i;

			if(velxD[index] == 0 || velyD[index] > winSize.width){
				continue;
			}
			if(abs(velxD[index]*velyD[index]) < minVel){
				continue;
			}
			if(abs(velxD[index]*velyD[index]) > maxVel){
				continue;
			}
			if(abs(velxD[index]*velyD[index]) > maxFoundVel){
				maxFoundVel = abs(velxD[index]*velyD[index]);
			}

			lineOrigin.x = (int) (boxCenter.x - (arrowLength/2)*velxD[index]*scaleFactor);
			lineOrigin.y = (int) (boxCenter.y - (arrowLength/2)*velyD[index]*scaleFactor);
			lineEnd.x = (int) (boxCenter.x + (arrowLength/2)*velxD[index]*scaleFactor);
			lineEnd.y = (int) (boxCenter.y + (arrowLength/2)*velyD[index]*scaleFactor);

			cvDrawLine(currentFrame,lineOrigin,lineEnd,colour,1);
			//cvDrawCircle(currentFrame,lineOrigin,5,cvScalar(255,255,128),-1);

			theta = atan(((double) (lineEnd.y - lineOrigin.y))/(lineEnd.x - lineOrigin.x));

			if(lineEnd.x > lineOrigin.x){
				theta = theta + pi;
			}
			//side 1
			phi = (theta + alpha + pi);
			arrowHeadEnd = cvPoint(lineEnd.x + r*cos(phi),lineEnd.y + r*sin(phi));
			cvDrawLine(currentFrame,lineEnd,arrowHeadEnd,colour,1);

			//side 2
			phi = (theta - alpha + pi);
			arrowHeadEnd = cvPoint(lineEnd.x + r*cos(phi),lineEnd.y + r*sin(phi));
			cvDrawLine(currentFrame,lineEnd,arrowHeadEnd,colour,1);
		}
	}
	//printf("max arrow value is %f\n",maxFoundVel);
}
void getBackground(CvCapture *capture){

	IplImage *frame;
	IplImage *bwFrame;
	IplImage *bwF;

	cvSetCaptureProperty(capture,CV_CAP_PROP_POS_FRAMES,400);

	int start_frame = 400;
	int total_frames = (int) cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_COUNT);

	if(start_frame == total_frames){
		//Need at least one background frame.
		return;
	}

	for(int frame_number = start_frame; frame_number < total_frames; frame_number++){
		frame = cvQueryFrame(capture);

		if(frame_number == start_frame){
			aveX = cvCreateImage(cvSize(frame->width,frame->height),IPL_DEPTH_32F,1);
			//minX = cvCreateImage(cvSize(frame->width,frame->height),IPL_DEPTH_8U,1);
			//maxX = cvCreateImage(cvSize(frame->width,frame->height),IPL_DEPTH_8U,1);
			bwFrame = cvCreateImage(cvSize(frame->width,frame->height),IPL_DEPTH_8U,1);
			bwF = cvCreateImage(cvSize(frame->width,frame->height),IPL_DEPTH_32F,1);

			//cvSet(minX,cvScalar(255));
			//cvZero(maxX);
		}
		cvCvtColor(frame,bwFrame,CV_BGR2GRAY);
		cvCvtScale(bwFrame,bwF);
		//cvMin(minX,bwFrame,minX);
		//cvMax(maxX,bwFrame,maxX);

		cvAdd(bwF,aveX,aveX);
	}

	IplImage *divisor = cvCreateImage(cvGetSize(aveX),IPL_DEPTH_32F,1);
	cvSet(divisor,cvScalar(total_frames - start_frame));
	cvDiv(aveX,divisor,aveX);

	//Now Calculate the stdDev.
	int index;
	float *stdData = NULL;
	cvSetCaptureProperty(capture,CV_CAP_PROP_POS_FRAMES,400);
	for(int frame_number = start_frame; frame_number < total_frames; frame_number++){
		frame = cvQueryFrame(capture);


		cvCvtColor(frame,bwFrame,CV_BGR2GRAY);
		cvCvtScale(bwFrame,bwF);

		if(frame_number == start_frame){
			stdDev = cvCreateImage(cvGetSize(frame),IPL_DEPTH_32F,1);
			stdData = (float*) stdDev->imageData;
			cvZero(stdDev);
		}

		for(int i = 0; i < frame->width; i++){
			for(int j = 0; j < frame->height; j++){
				index = bwFrame->widthStep*j + i;
				stdData[index] += bwFrame->imageData[index];
			}
		}
	}

	for(int i = 0; i < stdDev->width; i++){
		for(int j = 0; j < stdDev->height; j++){
			index = bwFrame->widthStep*j + i;
			stdData[index] = sqrt(stdData[index]/((float) (total_frames - start_frame)));
		}
	}


	cvReleaseImage(&bwFrame);
	cvReleaseImage(&bwF);
}
void removeBackground(IplImage *image){

	int index,stdIndex;
	float *aveData = (float*) (aveX->imageData);
	float *stdData = (float*) (stdDev->imageData);

	for(int i = 0; i < image->width; i++){
		for(int j = 0; j < image->height; j++){
			index = image->nChannels*(j*(image->width) + i);
			stdIndex = stdDev->width*j + i;

			if(image->imageData[index] > aveData[stdIndex] - stdData[stdIndex] && image->imageData[index] < aveData[stdIndex] + stdData[stdIndex]){
				setBlack(image,i,j);
			}
		}
	}
}
void setBlack(IplImage *image, int i, int j){
	char *data = image->imageData;
	int index = j*image->widthStep + i*image->nChannels;
	for(int k = 0; k < image->nChannels; k++){
		data[index + k] = 0;
	}
}
void drawAverage(IplImage *currentFrame, IplImage *velx, IplImage *vely, CvSize winSize){

	double x = 0, y = 0;
	double xDot = 0, yDot = 0;
	float *vxData = (float*) velx->imageData;
	float *vyData = (float*) vely->imageData;
	double minVal = 12;
	int minResults = 12;
	int totalResults = 0;
	double velocity;

	int index;
	for(int i = 0; i < velx->width; i++){
		for(int j = 0; j < velx->height; j++){
			index = j*velx->width + i;
			velocity = sqrt((vxData[index])*(vxData[index]) + (vyData[index])*(vyData[index]));

			if(velocity > minVal){
				totalResults++;
				x += winSize.width*i;
				y += winSize.height*j;

				xDot += (vxData[index]);
				yDot += (vyData[index]);
			}
		}
	}

	CvPoint lineOrigin;
	CvPoint lineEnd;
	CvPoint boxCenter;
	double scaleFactor = 0.2;
	double arrowLength = 10;

	if(totalResults > minResults){
		x = x/totalResults;
		y = y/totalResults;
		xDot = xDot/totalResults;
		yDot = yDot/totalResults;

		//Display the arrow.
		boxCenter.x = (int) x;
		boxCenter.y = (int) y;

		lineOrigin.x = (int) (boxCenter.x - (arrowLength/2)*xDot*scaleFactor);
		lineOrigin.y = (int) (boxCenter.y - (arrowLength/2)*yDot*scaleFactor);
		lineEnd.x = (int) (boxCenter.x + (arrowLength/2)*xDot*scaleFactor);
		lineEnd.y = (int) (boxCenter.y + (arrowLength/2)*yDot*scaleFactor);

		cvDrawLine(currentFrame,lineOrigin,lineEnd,cvScalar(255,255,128),3);
		cvDrawCircle(currentFrame,lineOrigin,5,cvScalar(255,255,128),-1);
	}
}
void findTopPoint(IplImage *image, IplImage *velx, IplImage *vely, CvSize winSize){
	//Find the average position.
	double x = 0, y = 0;
	double xDot = 0, yDot = 0;
	float *vxData = (float*) velx->imageData;
	float *vyData = (float*) vely->imageData;
	double minVal = 0;
	//int minResults = 12;
	int totalResults = 0;
	double velocity;
	static bool firstRun = true;
	static CvPoint lastPoint;
	static CvPoint lastHandPoint;
	bool pressed = false;
	CvPoint foundPoint;

	int index;
	for(int i = 0; i < velx->width; i++){
		for(int j = 0; j < velx->height; j++){
			index = j*velx->width + i;
			velocity = sqrt((vxData[index])*(vxData[index]) + (vyData[index])*(vyData[index]));

			if(velocity > minVal){
				totalResults++;
				x += winSize.width*i;
				y += winSize.height*j;

				xDot += (vxData[index]);
				yDot += (vyData[index]);
			}
		}
	}
	if(totalResults > 0){
		x = x/totalResults;
		y = y/totalResults;
	}

	if(!firstRun && totalResults == 0){
		x = lastHandPoint.x;
		y = lastHandPoint.y;
	}

	int counter = 0;
	if((totalResults > 0 || !firstRun) && !isBlack(image,x,y)){
		if(totalResults > 0){
			lastHandPoint = cvPoint((int) x, (int) y);
		}
		//Find the end points.
		CvPoint leftPoint = cvPoint((int) x,(int) y);
		CvPoint rightPoint = cvPoint((int) x,(int) y);

		bool atTop = false;
		bool incremented = false;
		while(!atTop){
			atTop = true;
			for(; !isBlack(image,leftPoint.x,leftPoint.y) && leftPoint.x > 0; leftPoint.x--){
			}
			for(; !isBlack(image,rightPoint.x,rightPoint.y) && rightPoint.x < image->width - 1; rightPoint.x++){
			}

			while(!isBlackLine(image,leftPoint,rightPoint) && (leftPoint.y > 0)){
				incremented = true;
				leftPoint.y--;
				rightPoint.y--;
			}
			if(leftPoint.y == 0){
				break;
			}
			if(!isBlack(image,leftPoint.x,leftPoint.y + 1) && incremented){
				rightPoint = leftPoint;
				leftPoint.y++;
				rightPoint.y++;
				atTop = false;
				incremented = false;
			}
			if(!isBlack(image,rightPoint.x,rightPoint.y + 1) && incremented){
				leftPoint = rightPoint;
				leftPoint.y++;
				rightPoint.y++;
				atTop = false;
				incremented = false;
			}
			if(counter >= image->height){
				printf("Had to break loop in findTopPoint.\n");
				printf("leftPoint.x is %d.\n",leftPoint.x);
				printf("leftPoint.y is %d.\n",leftPoint.y);
				break;
			}
			counter++;
		}

		int aveX = 0;
		int totalPoints = 0;
		for(int i = leftPoint.x; i <= rightPoint.x; i++){
			if(!isBlack(image,i,leftPoint.y + 5)){
				aveX += i;
				totalPoints++;
			}
		}
		if(totalPoints > 0){
			aveX = aveX/totalPoints;
		}
		foundPoint = cvPoint(aveX,leftPoint.y);
		if(firstRun){
			firstRun = false;
			lastPoint = foundPoint;
		}

		//Now draw the point.
		if(totalPoints > 0){
			pressed = fingerPressed(image,foundPoint);
			if(pressed){
				sendPoint(foundPoint,image,2);
			}
			else{
				sendPoint(foundPoint,image,1);
			}
			lastPoint = foundPoint;
		}
		else{
			sendPoint(lastPoint,image,0);
		}
	}
	else{
		sendPoint(lastPoint,image,0);
	}
}
void findTopOfFinger(IplImage *image){

	//printf("Running.\n");
	cvThreshold(image,image,24,0,CV_THRESH_TOZERO);
	CvPoint seed = getAverageLoc(image);

	if(seed.x != 0){
		CvPoint leftPoint = seed;
		CvPoint rightPoint = seed;

		for(; !isBlack(image,leftPoint.x,leftPoint.y) && leftPoint.x > 0; leftPoint.x--){
		}
		for(; !isBlack(image,rightPoint.x,rightPoint.y) && rightPoint.x < image->width - 1; rightPoint.x++){
		}
		while(!isBlackLine(image,leftPoint,rightPoint) && (leftPoint.y > 0)){
			leftPoint.y--;
			rightPoint.y--;
		}

		int aveX = 0;
		int totalPoints = 0;
		for(int i = leftPoint.x; i < rightPoint.x; i++){
			if(!isBlack(image,i,leftPoint.y + 1)){
				aveX += i;
				totalPoints++;
			}
		}
		if(totalPoints > 0){
			aveX = aveX/totalPoints;
		}

		//Now draw the point.
		if(totalPoints > 0){
			//cvDrawCircle(image,cvPoint(aveX,leftPoint.y),5,cvScalar(255,255,128),-1);
			if(fingerPressed(image,cvPoint(aveX,leftPoint.y - 1))){
				//cvDrawCircle(image,cvPoint(aveX,leftPoint.y - 1),5,cvScalar(255,255,128),-1);
			}
			else{
				//cvDrawCircle(image,cvPoint(aveX,leftPoint.y - 1),5,cvScalar(255,255,128),1);
			}

			printf("Drawing circle at x,y = %d,%d. \n",aveX,leftPoint.y);
			//sendPoint(cvPoint(aveX,leftPoint.y));
		}
		else{
			printf("Not drawing point.\n");
		}
	}
	else{
		printf("Seed is 0.\n");
	}

}
bool isBlackLine(IplImage *image, CvPoint leftPoint, CvPoint rightPoint){
	//Only for horizontal lines.

	bool vertical = (leftPoint.x == rightPoint.x);

	if(!vertical){
		for(int i = leftPoint.x; i < rightPoint.x; i++){
			if(!isBlack(image,i,leftPoint.y)){
				return(false);
			}
		}
	}
	return(true);
}
bool isBlack(IplImage *image,int i, int j){
	//Helper Method
	int index = (image->nChannels)*i + j*(image->widthStep);

	for(int k = 0; k < image->nChannels; k++){
		if((unsigned char) (image->imageData[index + k]) != 0){
			return(false);
		}
	}
	return(true);
}
void mouseDown(int Event, int x, int y, int flags, void* param){

	if(Event == CV_EVENT_MOUSEMOVE){
		mouseLocation = cvPoint(x,y);
		//printf("Moving mouse.");
	}
}
void displayResults(IplImage *image){

	CvFont font;
	CvScalar colour = cvScalar(255,0,0);
	CvPoint position = cvPoint(20,50);
	CvPoint intensityLocation = cvPoint(20,20);
	char pos[10];
	char intVal[10];
	unsigned char intensity = (unsigned char) (getColour(image,mouseLocation.x,mouseLocation.y).val[0]);

	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.4,0.5,0,1,9);
	sprintf(pos,"%d %d",mouseLocation.x,mouseLocation.y);
	sprintf(intVal,"%d",(int) intensity);
	cvPutText(image,pos,position,&font,colour);
	cvPutText(image,intVal,intensityLocation,&font,colour);
}
CvScalar getColour(IplImage *image,int i, int j){
	//Similar to cvGet2D except that it works on multichannel iamages.
	//Gets the colour of pixel at point i,j.
	//Some openCV built in methods use a colour as a cvScalar.
	//QAC = 2
	CvScalar colour;
	int index = j*image->widthStep + image->nChannels*i;
	for(int k = 0; k < image->nChannels && k < 4; k++){
		colour.val[k] = image->imageData[index + k];
	}
	return(colour);
}
IplImage *grabImage(){

	/*
	//FlyCaptureError error;
	FlyCaptureImage image;
	FlyCaptureImage convertedImage;

	//flycaptureGrabImage2(Dfly,&image);
	convertedImage.pData = new unsigned char[image.iRows*image.iCols*4];
	convertedImage.pixelFormat = FLYCAPTURE_BGRU;

	error = flycaptureConvertImage(Dfly,&image,&convertedImage);

	if(!img){
		img = cvCreateImageHeader(cvSize(image.iCols, image.iRows), IPL_DEPTH_8U, 4);
	}
	img->imageData = (char*) (convertedImage.pData);
	*/
	Error pgError;
	Image rawImage;
	pgError = Dfly.RetrieveBuffer(&rawImage);
	if (pgError != PGRERROR_OK){
		printf("Error in grabing frame.\n");
	}
	//printf("pixel format is %d.\n",rawImage.GetPixelFormat());
	IplImage *image = NULL;
	//Image convertedImage;
	//rawImage.Convert(PIXEL_FORMAT_BGR,&convertedImage);
	//image = convertImageToOpenCV(&convertedImage);
	image = convertImageToOpenCV(&rawImage);

	if(!image){
		printf("Failed to get image.\n");
	}
	//cvSaveImage("testing.bmp",image);
	return(image);
}
void initiateCamera(){
	//Starts the camera.

	/*
	//CameraGUIContext Dflygui;
	FlyCaptureError error;
	//CameraGUIError guiError;
	//FlyCaptureImage convertedImage;

	//This should start the video capture process
	error = flycaptureCreateContext(&Dfly);
	printError(error);
	error = flycaptureInitialize(Dfly,0);//Since this is the only device on the bus, second number is 0
	printError(error);
	error = flycaptureStart(Dfly,FLYCAPTURE_VIDEOMODE_800x600RGB,FLYCAPTURE_FRAMERATE_15);
	printError(error);
	*/
	BusManager busManager;
	unsigned int totalCameras;
	busManager.GetNumOfCameras(&totalCameras);
	printf("Found %d cameras on the bus.\n",totalCameras);
	busManager.GetCameraFromIndex(0, &gui);
	Error pgError;
	pgError = Dfly.Connect(&gui);
	if (pgError != PGRERROR_OK){
		printf("Error in starting the camera.\n");
	}
	Dfly.StartCapture();
	if (pgError != PGRERROR_OK){
		printf("Error in starting the camera capture.\n");
	}

}
IplImage *convertImageToOpenCV(Image* pImage){
	//Copied directly from movid.

	IplImage* cvImage = NULL;
	bool bColor = true;
	CvSize mySize;
	mySize.height = pImage->GetRows();
	mySize.width = pImage->GetCols();
	//cout << "Pixel format is " << pImage->GetPixelFormat() << endl;

	switch ( pImage->GetPixelFormat() )
	{
	case PIXEL_FORMAT_MONO8:
		cvImage = cvCreateImageHeader(mySize, 8, 1 );
		cvImage->depth = IPL_DEPTH_8U;
		cvImage->nChannels = 1;
		bColor = false;
		break;
	case PIXEL_FORMAT_411YUV8:
		cvImage = cvCreateImageHeader(mySize, 8, 3 );
		cvImage->depth = IPL_DEPTH_8U;
		cvImage->nChannels = 3;
		break;
	case PIXEL_FORMAT_422YUV8:
		cvImage = cvCreateImageHeader(mySize, 8, 3 );
		cvImage->depth = IPL_DEPTH_8U;
		cvImage->nChannels = 3;
		//printf("Inside switch.  Depth is %d\n",cvImage->depth);
		break;
	case PIXEL_FORMAT_444YUV8:
		cvImage = cvCreateImageHeader(mySize, 8, 3 );
		cvImage->depth = IPL_DEPTH_8U;
		cvImage->nChannels = 3;
		break;
	case PIXEL_FORMAT_RGB8:
		cvImage = cvCreateImageHeader(mySize, 8, 3 );
		cvImage->depth = IPL_DEPTH_8U;
		cvImage->nChannels = 3;
		break;
	case PIXEL_FORMAT_MONO16:
		cvImage = cvCreateImageHeader(mySize, 16, 1 );
		cvImage->depth = IPL_DEPTH_16U;
		cvImage->nChannels = 1;
		bColor = false;
		break;
	case PIXEL_FORMAT_RGB16:
		cvImage = cvCreateImageHeader(mySize, 16, 3 );
		cvImage->depth = IPL_DEPTH_16U;
		cvImage->nChannels = 3;
		break;
	case PIXEL_FORMAT_S_MONO16:
		cvImage = cvCreateImageHeader(mySize, 16, 1 );
		cvImage->depth = IPL_DEPTH_16U;
		cvImage->nChannels = 1;
		bColor = false;
		break;
	case PIXEL_FORMAT_S_RGB16:
		cvImage = cvCreateImageHeader(mySize, 16, 3 );
		cvImage->depth = IPL_DEPTH_16U;
		cvImage->nChannels = 3;
		break;
	case PIXEL_FORMAT_RAW8:
		cvImage = cvCreateImageHeader(mySize, 8, 3 );
		cvImage->depth = IPL_DEPTH_8U;
		cvImage->nChannels = 3;
		break;
	case PIXEL_FORMAT_RAW16:
		cvImage = cvCreateImageHeader(mySize, 8, 3 );
		cvImage->depth = IPL_DEPTH_8U;
		cvImage->nChannels = 3;
		break;
	case PIXEL_FORMAT_MONO12:
		//"Image format is not supported by OpenCV"
		bColor = false;
		break;
	case PIXEL_FORMAT_RAW12:
		//"Image format is not supported by OpenCV
		break;
	case PIXEL_FORMAT_BGR:
		cvImage = cvCreateImageHeader(mySize, 8, 3 );
		cvImage->depth = IPL_DEPTH_8U;
		cvImage->nChannels = 3;
		break;
	case PIXEL_FORMAT_BGRU:
		cvImage = cvCreateImageHeader(mySize, 8, 4 );
		cvImage->depth = IPL_DEPTH_8U;
		cvImage->nChannels = 4;
		break;
	case PIXEL_FORMAT_RGBU:
		cvImage = cvCreateImageHeader(mySize, 8, 4 );
		cvImage->depth = IPL_DEPTH_8U;
		cvImage->nChannels = 4;
		break;
	default:
		//ERROR in detecting image format
		//printf("Error, returning null.\n");
		return NULL;
	}

	if(bColor)
	{
		Image colorImage; //new image to be referenced by cvImage
		colorImage.SetData(new unsigned char[pImage->GetCols() * pImage->GetRows()*3], pImage->GetCols() * pImage->GetRows() * 3);
		pImage->Convert(PIXEL_FORMAT_BGR, &colorImage); //needs to be as BGR to be saved
		cvImage->width = colorImage.GetCols();
		cvImage->height = colorImage.GetRows();
		cvImage->widthStep = colorImage.GetStride();
		cvImage->origin = 0; //interleaved color channels
		cvImage->imageDataOrigin = (char*)colorImage.GetData(); //DataOrigin and Data same pointer, no ROI
		cvImage->imageData         = (char*)(colorImage.GetData());
		cvImage->widthStep              = colorImage.GetStride();
		cvImage->nSize = sizeof (IplImage);
		cvImage->imageSize = cvImage->height * cvImage->widthStep;
		//printf("Inside if.  width is %d, height is %d\n",cvImage->width,cvImage->height);
	}
	else
	{
		cvImage->imageDataOrigin = (char*)(pImage->GetData());
		cvImage->imageData         = (char*)(pImage->GetData());
		cvImage->widthStep         = pImage->GetStride();
		cvImage->nSize             = sizeof (IplImage);
		cvImage->imageSize         = cvImage->height * cvImage->widthStep;
		//at this point cvImage contains a valid IplImage
	}

	//printf("Pixel value at point 0,0 is %d.\n",(int) (cvImage->imageData[0]));
	return cvImage;
}
CvPoint getAverageLoc(IplImage *image){

	CvPoint average;
	int x = 0,y = 0, count = 0;


	for(int i = 0; i < image->width; i++){
		for(int j = 0; j < image->height; j++){
			if(!isBlack(image,i,j)){
				x += i;
				y += j;
				count++;
			}
		}
	}

	if(count > 0){
		average.x = x/count;
		average.y = y/count;
	}
	else{
		average.x = 0;
		average.y = 0;
	}
	return(average);
}
void sendPoint(CvPoint point,IplImage *image, int state){

	static int currentState = 0;
	static int newState = 0;
	static int newStateCounter = 0;
	int framesForStateChange = 3;
	int previousState = 0;
	bool stateChange = false;

	if(state == currentState && state != newState){
		newState = state;
		newStateCounter = 0;
	}
	else{
		if(state != currentState && state == newState){
			newStateCounter++;
			if(newStateCounter >= framesForStateChange){
				previousState = currentState;
				currentState = state;
				stateChange = true;
			}
		}
		else{
			newState = state;
			newStateCounter = 0;
		}
	}

	TuioTime time = TuioTime::getSessionTime();
	if(currentState != 0){
		tuioServer->initFrame(time);
	}

	if(stateChange){
		if(previousState == 1){
			printf("Removing tuio object.\n");
			tuioServer->removeTuioObject(hover);
		}
		if(previousState == 2){
			tuioServer->removeTuioCursor(cursor);
		}
		if(currentState == 1){
			hover = tuioServer->addTuioObject(1,(image->width - (float) (point.x) - 1)/image->width,((float) (point.y))/image->height,0);
		}
		if(currentState == 2){
			cursor = tuioServer->addTuioCursor((image->width - (float) (point.x) - 1)/image->width,((float) (point.y))/image->height);
		}
	}
	else{
		if(currentState == 1){
			//Update object
			tuioServer->updateTuioObject(hover,(image->width - (float) (point.x) - 1)/image->width,((float) (point.y))/image->height,0);
		}
		if(currentState == 2){
			//Update cursor
			tuioServer->updateTuioCursor(cursor,(image->width - (float) (point.x) - 1)/(image->width),((float) (point.y))/(image->height));
		}
	}

	if(currentState != 0 || stateChange){
		//printf("Sending (x,y) = %f,%f. \n",(image->width - (float) (point.x) - 1)/image->width,((float) (point.y))/image->height);
		tuioServer->commitFrame();
	}

	//Now draw a circle.
	if(currentState == 1){
		//cvDrawCircle(image,point,5,cvScalar(255,255,128),1);
	}
	if(currentState == 2){
		//cvDrawCircle(image,point,5,cvScalar(255,255,128),-1);
	}

	/*
	tuioServer->initFrame(time);
	if(firstCursor){
		//printf("Adding cursor.  %f,%f\n",((float) (point.x))/(image->width),((float) (point.y))/(image->height));
		cursor = tuioServer->addTuioCursor(((float) (point.x))/image->width,((float) (point.y))/image->height);
		hover = tuioServer->addTuioObject(1,((float) (point.x))/image->width,((float) (point.y))/image->height,0);
		firstCursor = false;
	}
	else{
		//printf("Sending (x,y) = %f,%f. \n",(float) (point.x),(float) (point.y));
		//cursor->update(time,(float) (point.x),(float) (point.y));
		tuioServer->updateTuioCursor(cursor,((float) (point.x))/(image->width),((float) (point.y))/(image->height));
		tuioServer->updateTuioObject(hover,((float) (point.x))/image->width,((float) (point.y))/image->height,0);
	}
	//tuioServer->stopUntouchedMovingCursors();
	tuioServer->commitFrame();*/

}
void cropImage(IplImage *image){

	for(int i = 0; i < image->width; i++){
		for(int j = 0; j < image->height; j++){
			if(i < 100 || i >= image->width - 100 || j < 100 || j >= image->height - 100){
				setBlack(image,i,j);
			}
		}
	}
}
bool fingerPressed(IplImage *image, CvPoint fingerTip){
	//This goes down 10 pixels from the

	int distance = 10;
	int difference = 25;
	bool foundLine = false;

	if(fingerTip.y > image->height - distance){
		printf("Failed to test fingertip.\n");
		return(false);
	}

	//int maxValue = 0;
	CvScalar colour, lastColour;
	int maxColourDifference = 0;
	for(int y = fingerTip.y; y < fingerTip.y + distance; y++){
		colour = getColour(image,fingerTip.x,y);
		lastColour = getColour(image,fingerTip.x,y - 1);
		if(maxColourDifference < lastColour.val[0] - colour.val[0]){
			maxColourDifference = lastColour.val[0] - colour.val[0];
		}
		if(colour.val[0] > lastColour.val[0] + difference){
			foundLine = true;
		}
	}
	//if(!foundLine){
		//printf("Max difference is %d.\n", maxColourDifference);
	//}
	return(foundLine);
}
void createCalibrationMatrix(){
	//See the learning openCV book for information on how this works.
	//Some lines were taken from the book.

	const char *videoFilename = "calib.avi";
	CvCapture *video = NULL;
	video = cvCreateFileCapture(videoFilename);
	if(!video){
		//cout << "Error, cannot find chessboard video." << endl;
		printf("Cannot find chessboard video.\n");
		return;
	}

	int frame_skip = 4;
	int video_frames = cvGetCaptureProperty(video, CV_CAP_PROP_FRAME_COUNT);
	int total_frames = video_frames/frame_skip;

	int board_w = 6, board_h = 8;
	int board_points = board_w*board_h;
	CvSize board_size = cvSize(board_w,board_h);

	//Storage space.
	CvMat *image_points = cvCreateMat(total_frames*board_points,2,CV_32FC1);
	CvMat *object_points = cvCreateMat(total_frames*board_points,3,CV_32FC1);
	CvMat *point_counts = cvCreateMat(total_frames,1,CV_32SC1);
	CvMat *intrinsic_matrix = cvCreateMat(3,3,CV_32FC1);
	CvMat *distortion_coeffs = cvCreateMat(5,1,CV_32FC1);
	cvZero(intrinsic_matrix);
	cvZero(distortion_coeffs);

	CvPoint2D32f *corners = new CvPoint2D32f[board_points];

	IplImage *image = NULL;
	IplImage *grey_image = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);

	int successes = 0;
	int corner_count;
	int step;
	cvNamedWindow("window",CV_WINDOW_AUTOSIZE);
	for(int frame = 0; frame < video_frames; frame += frame_skip){

		cvSetCaptureProperty(video,CV_CAP_PROP_POS_FRAMES,frame);
		image = cvQueryFrame(video);
		if(!image){
			break;
		}
		int found = cvFindChessboardCorners(image,board_size,corners,&corner_count,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		//cout << "For frame " << frame << ", number of points found is " << corner_count << endl;

		cvDrawChessboardCorners(image,board_size,corners,corner_count,found);
		//cvSaveImage("image.bmp",image);
		cvWaitKey(10);
		cvShowImage("window",image);


		if(corner_count == board_points){
			step = successes*board_points;
			for(int i = step, j = 0; j < board_points; ++i, ++j){
				CV_MAT_ELEM(*image_points,float,i,0) = corners[j].x;
				CV_MAT_ELEM(*image_points,float,i,1) = corners[j].y;
				CV_MAT_ELEM(*object_points,float,i,0) = j/board_w;
				CV_MAT_ELEM(*object_points,float,i,1) = j % board_w;
				CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
			}
			CV_MAT_ELEM(*point_counts,int,successes,0) = board_points;
			successes++;
		}
	}
	//cout << "Number of successes is " << successes << endl;

	/*
	CvMat *object_points2 = cvCreateMat(successes*board_points,3,CV_32FC1);
	CvMat *image_points2 = cvCreateMat(successes*board_points,2,CV_32FC1);
	CvMat *point_counts2 = cvCreateMat(successes,1,CV_32SC1);

	for(int i = 0; i < successes*board_points; ++i){
		CV_MAT_ELEM(*image_points2,float,i,0) = CV_MAT_ELEM(*image_points,float,i,0);



	}*/

	if(successes != total_frames){
		//cout << "Error, did not find all the points on all the frames." << endl;
		//cout << "Successes is " << successes << ", total_frames = " << total_frames << endl;
		printf("Error, did not find all the points on all the frames.\n");
		printf("Successes is %d, total frames = %d.\n",successes,total_frames);
		return;
	}

	CV_MAT_ELEM(*intrinsic_matrix,float,0,0) = 1.0f;
	CV_MAT_ELEM(*intrinsic_matrix,float,1,1) = 1.0f;

	cvCalibrateCamera2(object_points,image_points,point_counts,cvGetSize(image),intrinsic_matrix,distortion_coeffs,NULL,NULL,0);

	cvSave("Intrinsics.xml",intrinsic_matrix);
	cvSave("Distortions.xml",distortion_coeffs);

	IplImage *mapx = cvCreateImage(cvGetSize(image),IPL_DEPTH_32F,1);
	IplImage *mapy = cvCreateImage(cvGetSize(image),IPL_DEPTH_32F,1);
	cvInitUndistortMap(intrinsic_matrix,distortion_coeffs,mapx,mapy);

	cvSetCaptureProperty(video,CV_CAP_PROP_POS_FRAMES,0);
	for(int frame = 0; frame < video_frames; frame++){
		image = cvQueryFrame(video);
		IplImage *image_clone = cvCloneImage(image);

		cvRemap(image,image_clone,mapx,mapy);
		cvShowImage("window",image_clone);

		cvWaitKey(20);

		cvReleaseImage(&image_clone);
	}


	cvReleaseImage(&grey_image);
	cvReleaseCapture(&video);
}
IplImage *calibrateImage(IplImage *image){

	IplImage *new_image;
	new_image = cvCloneImage(image);

	//new_image = cvCreateImage(cvSize(image->width*2,image->height*2),IPL_DEPTH_8U,image->nChannels);
	//cvZero(new_image);
	//IplImage *calib_image = cvCloneImage(new_image);
	//cvSetImageROI(new_image,cvRect(image->width/2,image->height/2,image->width,image->height));
	//cvCopy(image,new_image);

	//cvRemap(new_image,calib_image,mapx,mapy);
	cvRemap(image,new_image,mapx,mapy);

	//cvReleaseImage(&new_image);
	return(new_image);
}

