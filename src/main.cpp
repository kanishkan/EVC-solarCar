#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "config.h"
#include "serialCar.h" 
#include "trafficSignsDetection.h"

using namespace cv;
using namespace std;
Rect ROI;

// Input Configuration
#define VIDEO_INPUT 1
#define VIDEO_FILE 0
#define ADVANCE_CASE 0

// Power log
FILE *logFile = NULL;

// Global
unsigned int frameId = -1;
bool mtxCam = true;

// Serial Communication
int uart0_filestream;
char command[10]; //Format= {'0','0','1','T','+'};

//
void showIm(String title, Mat im);
Mat preProcess(Mat image);
void findLane(vector<Vec4i> *lines, int row, int col,std::vector<Lane> *lane);
void laneFilter(std::vector<Lane> lanes_org, int w, int h,\
			int *leftLane, int *rightLane, int *horizLane );
void filterLane_strength(std::vector<Lane> lanes, Mat edges, Mat raw);
void findSetPoint(Mat im_roi, std::vector<Lane> lanes_org, int leftLane, int rightLane, int horizLane, \
						TurnType *turn, CvPoint *setPoint, int w, int h);
void init();
void findBorderPoints(std::vector<Lane> lanes_org, int l, CvPoint *lowPt,CvPoint *highPt, int w, int h );
void updateTrackingInfo();

//print functions
void dir2str(DirectionVector dir, char *var);
void lanePat2str(LanePattern pat, char *var);
void pid2str(PIDRotation rot, char *var);
void printControlInfo(char *var, ControlInfo pt);
void conf2str(Confidence conf, char *var);
void dir2str(DirectionVector dir, char *var);

// Util functions
float dist(CvPoint p1, CvPoint p2);
CvPoint intersection(CvPoint o1, CvPoint p1, CvPoint o2, CvPoint p2);
struct ControlInfo previousFrame;
struct ControlInfo currentFrame;

int validResult = 0;

// Sign detection
void findSign(Mat src);
struct signInfo detectedSignInfo;
struct signInfo signWindow[SIGN_TEMPORAL_WIN];

// UART
void sendMove();
void sendRotate(float angle);
void sendCmd(float angle);

// Safety Layer
void sendCmd(float angle){
	//angle = angle/10;
	if(validResult != 0){
		printf("If case\n");
		//if(angle<LANE_ROTATION_ERROR_TH)
		//	angle = angle*1.5;	
		sendRotate(angle);
		printf("---------------------------------SERIAL DATA: %s\n",command);
		waitKey(500);
		if(fabs(angle)<12){
			sendMove();
			printf("---------------------------------SERIAL DATA: %s\n",command);
		}
		//sleep(1000);
		//sprintf(command,"05T+");
		//sendCommand((unsigned char*)command,uart0_filestream);
	}
	else{	// Unknown lane pattern
		printf("Else case\n");
		// Go straight
		sendMove();
		
		//waitKey(1000);
		//printf("MOve done\n");
		//sendRotate(-120.0);
		//waitKey(100);
		printf("---------------------------------SERIAL DATA: %s\n",command);
		//sendMove();
		//printf("---------------------------------SERIAL DATA: %s\n",command);
	}
}
void sendMove(){
	float dis = STEP_DIST;
	if((int)fabs(dis)>99)
		sprintf(command,"%dT%s",(int)fabs(dis), dis<0?"-":"+");
	else if((int)fabs(dis)>=10)
		sprintf(command,"0%dT%s",(int)fabs(dis), dis<0?"-":"+");
	else
		sprintf(command,"00%dT%s",(int)fabs(dis), dis<0?"-":"+");
	sendCommand((unsigned char*)command,uart0_filestream);
#if EN_PWR_LOG	
	logEnergy(uart0_filestream,logFile);
#endif
}
void sendRotate(float angle){
	if((int)fabs(angle)>99)
		sprintf(command,"%dR%s",(int)fabs(angle), angle<0?"-":"+");
	else if((int)fabs(angle)>=10)
		sprintf(command,"0%dR%s",(int)fabs(angle), angle<0?"-":"+");
	else
		sprintf(command,"00%dR%s",(int)fabs(angle), angle<0?"-":"+");
	sendCommand((unsigned char*)command,uart0_filestream);
#if EN_PWR_LOG	
	logEnergy(uart0_filestream,logFile);
#endif
}

int main(int argc, char** argv )
{
	int roi_rows, roi_cols;
	Mat image;
	init();

#if VIDEO_INPUT 
	VideoCapture cap(0); // open the default camera
    if(!cap.isOpened()){  // check if we succeeded
     	printf("Unable to open camera.!\n");
        return -1;
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT);
	// cap.set(CV_CAP_PROP_FPS,1);
	// cap.set(CV_CAP_PROP_BUFFERSIZE, 2);
	// int fps = cap.get(CV_CAP_PROP_FPS);
while(1){
    bool valD=false;
    //waitKey();
	//printf("Elapsed time: %fsec FPS: %d skip count: %d\n",t_cap,fps, dummyF);
	//for(int dummyF=0;dummyF<50;dummyF++){
	double t_cap = (double)getTickCount()/getTickFrequency();
#if 0
    cap >> image; 	//650ms same as cap.read(image);
#else
    cap.open(0);
  	cap >> image;
  	cap >> image;
  	cap.release();
	//cap.grab();		// 710ms
    //cap.retrieve(image);
#endif
    //	if(dummyF==25)
    //		valD = true;
    t_cap = ((double)getTickCount() - t_cap)/getTickFrequency();
    //	printf("Loop1-%d: %f\n",dummyF,t_cap);
    //	if(valD){
    //		 waitKey(10000);valD=false;
    //	}
	//}
	//cap >> image;
	//waitKey();
    printf("####     Capture Time : %fms\n",t_cap*1000);
    //t_cap = (double)getTickCount(); // To find elapsed frame

#elif VIDEO_FILE
	//VideoCapture cap("video_new.h264");
	VideoCapture cap("vid22_8_take1.h264");
	if(!cap.isOpened()){  // check if we succeeded
     	printf("Unable to open video file.!\n");
        return -1;
    }
	while(1)
	{
		waitKey(0);
		frameId++;
		Mat rawImg;
		for(int fRate=0;fRate<100;fRate++)
			cap >> image;
			//cap >> rawImg;
		if(image.empty())
            break;
		//resize(rawImg, image, Size(400, 300), 0, 0, INTER_CUBIC); 
		//resize(rawImg, image, Size(800, 600), 0, 0, INTER_CUBIC); 

#else
    if ( argc == 2 ){
         image = imread( argv[1], 1 );
		 printf("Input: %s\n", argv[1]);
	}
	else{
		char imgeLocation[100] = "raw_case/right.jpg";
		image = imread(imgeLocation,1);
	}
{
#endif
	// Validate Image
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
/*******************************************************************************************/

	// Pre-processing
    validResult = 0;
    // Pre-process - Lane detection
	Mat im_roi,im_edge;
	//showIm("Original", image);
	double t = (double)getTickCount();
	im_roi = preProcess(image);

	//showIm("Pre-process",im_roi);
	// Edge-detection
	//Canny(im_roi, im_edge, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);
	im_edge = im_roi;
	//if(EDGE_IMG) showIm("Image-Edges",im_edge);

	// Pre-Process - Sign Detection
	Mat src;
	//resize(image, src, Size(), 0.25, 0.25, INTER_LINEAR);
	src = image;
    findSign(src);

    //updateTrackingInfo();continue;
    //continue;
	
	// Hough Transform
	std::vector<Vec4i> lines;
	double rho = 1;
	double theta = CV_PI/360;
	HoughLinesP(im_edge, lines, rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);

/*******************************************************************************************/
// Lane detection Algorithm

	// Lane Classification
	roi_rows = im_roi.rows;
	roi_cols = im_roi.cols;
	std::vector<Lane> lane;
	findLane(&lines,roi_rows,roi_cols, &lane);

	// Lane filtering
	int leftLane, rightLane, horizLane;
	laneFilter(lane, roi_cols, roi_rows, &leftLane, &rightLane, &horizLane);

	// Detect Set-point
	enum TurnType turn;
	CvPoint setPoint;

	// Find control output
	/* 	1. If turn is critical, do immediately. Otherwise, modify the set point to mid value
		2. Find control in terms of angle 
	*/
	findSetPoint(im_roi, lane, leftLane, rightLane, horizLane, &turn, &setPoint, roi_cols, roi_rows);

	// Update Tracking Details
	/*	1. Store Current frame info. and reset the values
	*/
	updateTrackingInfo();
t = ((double)getTickCount() - t)/getTickFrequency();
printf("Elapased Time: %fms  frame rate: %f\n",t*1000, 1/t );
	// Command PID
	/* Communication protocol driver
	*/
}
#if EN_PWR_LOG
	fclose(logFile); 
#endif
//	waitKey(2000);
}

void findSign(Mat src){
 	traffic *trafficSignals;
    // init scan
    float areaV =0;
    DirectionVector dir = NONE;
    !SIGN_DBG?:printf("---------------------SIGN-start-%d--------------------------------\n",frameId);
    //#pragma openmp parallel
    //for(int i =0; i<3;i++)
    { 
        array_traffic r = detectObjects(src);
        trafficSignals        = r.signs;
        int noRed   = r.noSigns;
        for(int p=0;p<noRed;p++){
        	!SIGN_DBG?:printf("%d/%d -> %s\n",p+1,noRed,trafficSignals[p].classification);
            putText(src,trafficSignals[p].classification , cvPoint(trafficSignals[p].xpos,trafficSignals[p].ypos),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

            // Scan
            if(areaV<trafficSignals[p].area){
            	areaV = trafficSignals[p].area;
            	if(strcmp(trafficSignals[p].classification, "LEFT")==0)
					dir = LEFT;
				else if(strcmp(trafficSignals[p].classification, "RIGHT")==0)
					dir = RIGHT;
				else if(strcmp(trafficSignals[p].classification, "UTURN")==0)
					dir = UTURN;
				else if(strcmp(trafficSignals[p].classification, "STOP")==0)
					dir = STOP;
				else if(strcmp(trafficSignals[p].classification, "STRAIGHT")==0)
					dir = STRAIGHT;
				else
					dir = NONE;	//should not reach
            }
        }
    }
    // Update in global variable
    detectedSignInfo.direction = dir;
    detectedSignInfo.area = areaV;
    detectedSignInfo.confidanceCount = 0;
#if IMG_ON
    showIm("SIGN - Original Image",src);
//    imshow("w", src);dir;
#endif
    free(trafficSignals);

    // Closest sign
    char var[50];
    dir2str(dir,var);
    !SIGN_DBG?:printf("Closest Sign: %s, Area = %.2f calib:%.2f\n",var,areaV, (areaV==0)?0:(5000-areaV)/500);

    // Find control
	int confidence = 0;
	!SIGN_DBG?:printf("Sign array: ");
	for(int i=0;i<SIGN_TEMPORAL_WIN;i++){
		!SIGN_DBG?:printf("%d ",signWindow[i].direction);
		if(signWindow[i].direction == detectedSignInfo.direction)
			confidence = confidence+1;
	}
	!SIGN_DBG?:printf("\n");

	detectedSignInfo.confidanceCount = confidence;
	!SIGN_DBG?:printf("Conf count:%d\n",confidence);
	if(detectedSignInfo.confidanceCount >= SIGN_TEMPORAL_TH){ // Stringest Sign
		if(detectedSignInfo.direction == NONE){
			!SIGN_DBGC?:printf("SIGN_CON: No sign found.!\n");
		}
		else{
			!SIGN_DBGC?:printf("SIGN_CON:Sign = %d.!\n",detectedSignInfo.direction);
		}

	}else{
		!SIGN_DBGC?:printf("SIGN_CON:Weak sign.! Discarding...\n");
	}
	!SIGN_DBG?:printf("---------------------SIGN-end-%d----------------------------------\n",frameId);
}

CvPoint rightLaneLowPt, rightLaneHighPt;
CvPoint leftLaneLowPt, leftLaneHighPt;

void updateTrackingInfo(){
#if CONF_DBG
	printf("---------------------Update-start-%d\n",frameId);
	printControlInfo("previousFrame", previousFrame);
	printControlInfo("CurrentFrame", currentFrame);
#endif
	// Update sign
	for(int i=SIGN_TEMPORAL_WIN-2;i>=0;i--){
		// shift
		signWindow[i+1].direction 		= signWindow[i].direction;
		signWindow[i+1].area 			= signWindow[i].area;
		signWindow[i+1].confidanceCount = signWindow[i].confidanceCount;
	}
	signWindow[0].direction 		= detectedSignInfo.direction;
	signWindow[0].area 				= detectedSignInfo.area;
	signWindow[0].confidanceCount 	= detectedSignInfo.confidanceCount;
	!SIGN_DBG?:printf("New sign array: ");
	for(int i=0;i<SIGN_TEMPORAL_WIN;i++){
		!SIGN_DBG?:printf("%d ",signWindow[i].direction);
	}
	!SIGN_DBG?:printf("\n");

	// Control update
	bool signUpdate = false;
	// If previous frame is empty, load the sign
	if( previousFrame.nextDirection == NONE && detectedSignInfo.direction != NONE){
		signUpdate = true;
		currentFrame.nextDirection = detectedSignInfo.direction;
	}
	// If prevFrame is not empty, wait for getting close
	else if(detectedSignInfo.confidanceCount >= SIGN_TEMPORAL_TH && detectedSignInfo.direction != NONE && detectedSignInfo.direction != STRAIGHT){
		//printf("##################### Area:%f\n",detectedSignInfo.area);
		!SIGN_DBG?:printf("Confidence test passed.!\n");
		int th_count = SIGN_AREA_TH;
		if(detectedSignInfo.area >th_count){ // Close range
			currentFrame.nextDirection = detectedSignInfo.direction;
			signUpdate = true;
			!SIGN_DBGC?:printf("Close range sign.! Rnage:%.2f signUpdate:%B\n",detectedSignInfo.area,signUpdate);
		}
		else{
			!SIGN_DBGC?:printf("Sign is not in close range.! (%.2f/th_count)\n",detectedSignInfo.area);
		}
	}
	if(signUpdate){
		previousFrame.valid = 1;
		previousFrame.nextDirection = currentFrame.nextDirection;
		previousFrame.trafficSign = currentFrame.trafficSign;
	}
	//Update Lane
	else if(currentFrame.lanePatern==UNKNOWN || currentFrame.lanePatern==NOLANE ){
		// No lanes found; leave the previous pattern previous pattern and update its temportal location.
		!LANE_PATTERN_DBG?:printf("No lane found on current frame. So retaining Previous frame.\n");
		previousFrame.dirCounter = LOW;
	}
	else{
		!LANE_PATTERN_DBG?:printf("Current lane has valid pattern.\n");
		if(currentFrame.lanePatern==HORIZONTAL || currentFrame.lanePatern==CORNER || currentFrame.lanePatern==ERROR){ // Start as new since rotation is done
			!LANE_PATTERN_DBG?:printf("Pattern: Horiz/Cornor/error.\n");
			!LANE_PATTERN_DBG?:printf("Control: valid is set based on signUpdate.\n");
			//previousFrame.valid = signUpdate?1:0;
		}
		else{ // Valid pattern; update values
			!LANE_PATTERN_DBG?:printf("Pattern: Normal Pattern\n");
			previousFrame.direction = currentFrame.direction;         
			previousFrame.angle = currentFrame.angle;
			previousFrame.trafficSign = currentFrame.trafficSign;
			previousFrame.dirCounter = HIGH;
			previousFrame.signCounter = currentFrame.signCounter; // Not used yet
			previousFrame.valid = 1;
			previousFrame.lanePatern = currentFrame.lanePatern;
			previousFrame.nextDirection = currentFrame.nextDirection; // carry forward
			previousFrame.pidControlMsgType = currentFrame.pidControlMsgType;
		}
	}
#if CONF_DBG
	printControlInfo("new previousFrame", previousFrame);
	printf("---------------------Update-end-%d\n",frameId);
#endif
}
void findSetPoint(Mat im_roi, std::vector<Lane> lanes_org, int leftLane, int rightLane, int horizLane, \
						TurnType *turn, CvPoint *setPoint, int w, int h){
	/* Find set point based on left, right and horizontal lane
	Normal Case:
		1. No horizontal lane(previous and current frame):
			Decide control based on sign, vertical lanes(ditance to set-point, angle) and last position.
		2. One lane:
			Vertical/Horiz: Sign, past control(set-point)+lane angle*
		3. One Hor+One vert:
		4. Corner case:
		5. Broken line:
	*/
	//CvPoint horLaneLowPt, horLaneHighPt;

	// Mid-point in y direction for filtering
	int leftLineMidy = -1; // For left lane mid point filter
	int rightLineMidy = -1;

	// Calculate boarder Crossing points
	if(leftLane!=-1){
		!SETPT_DBG?:printf("LeftLane ");
		findBorderPoints(lanes_org, leftLane, &leftLaneLowPt, &leftLaneHighPt, w,h );
		leftLineMidy = (lanes_org[leftLane].p1.y+lanes_org[leftLane].p2.y)/2;
	}
	if(rightLane!=-1){
		!SETPT_DBG?:printf("RightLane ");
		findBorderPoints(lanes_org, rightLane, &rightLaneLowPt,&rightLaneHighPt,w,h);
		rightLineMidy = (lanes_org[rightLane].p1.y+lanes_org[rightLane].p2.y)/2;
	}
	// Find mid points
	int lowerMid,upperMid,horizMidx,horizMidy;
	int verticalLaneDist = 0;

	if(horizLane != -1){
		horizMidx = (lanes_org[horizLane].p1.x+lanes_org[horizLane].p2.x)/2;
		horizMidy = (lanes_org[horizLane].p1.y+lanes_org[horizLane].p2.y)/2;
	}
	if(leftLane!=-1){
		lowerMid= (rightLaneLowPt.x +leftLaneLowPt.x)/2;
		if(horizMidy > leftLineMidy){ // VK-extra case
		//	leftLane = -1;
			!SETPT_DBG?:printf("Left Lane is after Horizontal Lane.! Discard Left lane\n");
		}
	}
	if(rightLane!=-1){
		upperMid = (rightLaneHighPt.x +leftLaneHighPt.x)/2;
		if(horizMidy > rightLineMidy){	// VK-extra case
		//	rightLane = -1;
			!SETPT_DBG?:printf("Right Lane is after Horizontal Lane.! Discard Right lane\n");
		}
	}
	
	CvPoint p1,p2, interSectPt ;
	p1.x = p1.y = p2.x = p2.y =0;
	
	// Initialize default values
	currentFrame.lanePatern = NORMAL;
	int discardHorizLane = 0;
	int distToHorizLane = h;
	float angleOffset = 0;
	int validPrevFame = previousFrame.valid;

	// Case where horizontal line will not affect detection
	if(horizLane != -1){
		distToHorizLane = h-horizMidy;
		if(/*distToHorizLane<HORIZ_LANE_MIN_DIST ||*/ horizMidy<=HORIZ_LANE_LEFT_REJECTION || horizMidy>=HORIZ_LANE_RIGHT_REJECTION)			// Distance is high | Left border |Right border
			discardHorizLane = 1;
		!SETPT_DBG?:printf("Distance to horizontal: %d\n",distToHorizLane);
	}
	!SETPT_DBG?:printf(" Loop Start: horLane =%d, discard:%d    LeftLane:%d rightLane:%d\n",horizLane,discardHorizLane,leftLane,rightLane);
	if(horizLane== -1 || discardHorizLane ==1){		// Only vertical Lanes
		if(leftLane!=-1 && rightLane!=-1){		// Two Lanes
			if(leftLaneLowPt.x < rightLaneLowPt.x && leftLaneHighPt.x < rightLaneHighPt.x){ // Lane does not cross			
				verticalLaneDist = dist(rightLaneLowPt,leftLaneLowPt);
				if(verticalLaneDist<VERT_LANE_SEPERATION){
					// Lanes are too close # Not exected
					!SETPT_DBG?:printf("Vertical Lanes are too close.\n");
					currentFrame.lanePatern = UNKNOWN;
				}else{
					!SETPT_DBG?:printf("Vertical Lanes : Strong values\n");
					p1.x = lowerMid;
					p1.y = h;
					p2.x = upperMid;
					p2.y = 0;
					currentFrame.lanePatern = NORMAL;
					validResult = 1;
				}
			}
			// Lane crossings
			else{
				!SETPT_DBG?:printf("Predicited Lanes crossing.!\n");
				interSectPt = intersection(leftLaneLowPt, leftLaneHighPt, rightLaneLowPt, rightLaneHighPt);
				if(interSectPt.x==0 && interSectPt.y==0){
					!SETPT_DBG?:printf("Could not find intersect point.!\n");
					currentFrame.lanePatern = UNKNOWN;
				}
				else{
					!SETPT_DBG?:printf("Vertical Line intersect point: (%d,%d).!\n",interSectPt.x,interSectPt.y);	
					p1.x = lowerMid;
					p1.y = h;
					p2.x = interSectPt.x;
					p2.y = interSectPt.y;
					currentFrame.lanePatern = CORNER;
					validResult = 1;
				}
			}
		}else{ // One Lane
			p1.x = w/2;
			p1.y = h;
			p2.x = w/2;
			p2.y = 0;
			if(leftLane!=-1){	// Left Lane alone
				!SETPT_DBG?:printf("Single Left Lane- Angle:%f\n",lanes_org[leftLane].angle);
				//angleOffset = 45-lanes_org[leftLane].angle;
				angleOffset = -1*(SS_LANE_ANGLE+lanes_org[leftLane].angle);  //try to keep steady angle
				//p2.x = leftLaneHighPt;
				//p2.y = 0;
				currentFrame.lanePatern = L_LANE;
				validResult = 1;
			}
			else if(rightLane!=-1){
				!SETPT_DBG?:printf("Single Right Lane- Angle:%f\n",lanes_org[rightLane].angle);
				//p2.x = rightLaneHighPt;
				//p2.y = 0;
				//angleOffset = lanes_org[rightLane].angle-45;
				angleOffset = (SS_LANE_ANGLE-lanes_org[rightLane].angle);
				currentFrame.lanePatern = R_LANE;
				validResult = 1;
			}
			else{	// No vertical Lane
				!SETPT_DBG?:printf("No Vetical Lanes.\n" );
				currentFrame.lanePatern = NOLANE;
			}
		}
	}else if(horizLane != -1){		// Horizontal and Verival Lane
		// Guard condition for corner case
		// If horizontal line is before vertical, then discard vertical lines.
		// TODO

		// Distance to horizontal Lane = horizMidy
		distToHorizLane = h-horizMidy;	
		if(leftLane!=-1 && rightLane!=-1){			// Two vertical and one horizontal
			//Set intersect point to mid of two vertical lanes
			!SETPT_DBG?:printf("Horizontal Lane + 2 vertical lane.\n" );
			CvPoint leftLInt = intersection(leftLaneLowPt, leftLaneHighPt,lanes_org[horizLane].p1,lanes_org[horizLane].p2 );
			CvPoint rightLInt = intersection(rightLaneLowPt, rightLaneHighPt,lanes_org[horizLane].p1,lanes_org[horizLane].p2);
			p2.x = (leftLInt.x+rightLInt.x)/2;
			p2.y = (leftLInt.y+rightLInt.y)/2;
			validResult = 1;
		}else{	
			if(leftLane!=-1){	// Left Lane alone
				!SETPT_DBG?:printf("Horizontal + Signle Left Lane- Angle:%f\n",lanes_org[rightLane].angle);
				CvPoint leftLInt = intersection(leftLaneLowPt, leftLaneHighPt,lanes_org[horizLane].p1,lanes_org[horizLane].p2 );
				!SETPT_DBG?:printf("Horizontal+Left: Intersect:(%d,%d)\n",leftLInt.x,leftLInt.y);
				int leftMidx= (lanes_org[leftLane].p1.y + lanes_org[leftLane].p2.y)/2;
				if(leftLInt.y<= h/2){ // Avoid sharp ende on close
					!SETPT_DBG?:printf("Not a sharp edge.!\n");
					//printf("LOG: %d,%d  %d:%d\n",leftLInt.x,leftLInt.y,horizMidx,horizMidy);
					//printf("LOG: horiz mid: %d, leftMid:%d\n",horizMidy,leftMidx);
					if(leftMidx > horizMidy){
						p2.x = (leftLInt.x +horizMidx )/2;
						p2.y = (leftLInt.y +horizMidy )/2;
					}
					else currentFrame.lanePatern = HORIZONTAL;
				}else{	// 
					!SETPT_DBG?:printf("Sharp edge.! Avoid maximum value.\n");
					
					//printf("LOG: horiz mid: %d, leftMid:%d\n",horizMidy,leftMidx);
					if(leftMidx < horizMidy){
						currentFrame.lanePatern = HORIZONTAL;
					}
					else if(lanes_org[horizLane].p1.x>lanes_org[horizLane].p2.x){
						p2.x = (leftLInt.x + lanes_org[horizLane].p1.x )/2;
						p2.y = (leftLInt.y + lanes_org[horizLane].p1.y )/2;
					}
					else{
						//printf("Smooth edge.!\n" );
						p2.x = (leftLInt.x + lanes_org[horizLane].p2.x )/2;
						p2.y = (leftLInt.y + lanes_org[horizLane].p2.y )/2;
					}
				}
				validResult = 1;
				currentFrame.lanePatern = L_LANE;
			}
			else if(rightLane!=-1){
				!SETPT_DBG?:printf("Horizontal + Signle Right Lane- Angle:%f\n",lanes_org[rightLane].angle);
				CvPoint rightLInt = intersection(leftLaneLowPt, leftLaneHighPt,lanes_org[horizLane].p1,lanes_org[horizLane].p2 );
				!SETPT_DBG?:printf("Horizontal+Right: Intersect:(%d,%d)\n",rightLInt.x,rightLInt.y);
				if(rightLInt.y<= h/2){ // Avoid sharp ende on close
					!SETPT_DBG?:printf("Not a sharp edge.!\n");
					//p2.x = (rightLInt.x +horizMidx )/2;
					//p2.y = (rightLInt.y +horizMidy )/2;
					currentFrame.lanePatern = HORIZONTAL;
				}else{	// 
					!SETPT_DBG?:printf("Sharp edge.! Avoid maximum value.\n");
					if(lanes_org[horizLane].p1.x<lanes_org[horizLane].p2.x){
						p2.x = (rightLInt.x + lanes_org[horizLane].p1.x )/2;
						p2.y = (rightLInt.y + lanes_org[horizLane].p1.y )/2;
					}
					else{
						p2.x = (rightLInt.x + lanes_org[horizLane].p2.x )/2;
						p2.y = (rightLInt.y + lanes_org[horizLane].p2.y )/2;
					}
				}
				validResult = 1;
				currentFrame.lanePatern = R_LANE;
			}
			else{	// Only Horizontal Lane
				!SETPT_DBG?:printf("Only Horizontal Lanes.! Previous frame: %s \n", (validPrevFame==1)?"Valid(Control:Prev.frame)":"No(Control:Random)"); 
				currentFrame.lanePatern = HORIZONTAL;
			}
		}	
	}
	else{
		!CONT_DBG?:printf(" No Lanes.. :(\n");
		currentFrame.lanePatern = NOLANE;
	}

	
	

	//line(im_roi, p1, p2, CV_RGB(200, 10, 10), 2, 8);
	//printf("Projection Line: (%d,%d) (%d,%d) \n",p1.x,p1.y,p2.x,p2.y);
	//sprintf(result,"%s","Projected Line");
	//putText(im_roi, result, p1, FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255,255), 1, CV_AA);
	// Load last frame info:


	if(currentFrame.lanePatern == CORNER){	// Do 135 degree turn
		//Take Turn based on intersect point
		!CONT_DBG?:printf("Corner Case-> Adding roation.! Type: ");
		if(interSectPt.x <= w/2 ){
			angleOffset = 135;
			!CONT_DBG?:printf("Left..") ;
		}
		else{
			angleOffset = -135;
			!CONT_DBG?:printf("Right..");
		}
		
		// Clear history if it is corner case
		if(previousFrame.nextDirection == STOP){
			printf("Stop Sign detected earlier.! Stoping Execution.!\n");
			exit(0);
		}else if(previousFrame.nextDirection == UTURN){
			printf("U-Turn Executing.!\n");
			previousFrame.nextDirection = NONE;
		}
	}

//	else if(currentFrame.lanePatern == HORIZONTAL){	
	else if(currentFrame.lanePatern == HORIZONTAL && distToHorizLane<=HORIZ_LANE_CRITICAL_DIST){		// Immediate response
		currentFrame.pidControlMsgType = ROTATE;
		!CONT_DBG?:printf("Close Horizontal.! Adding rotation.!\n");
	}
	//else printf("distToHorizLane:%d\n",distToHorizLane);
		/* Not checking for length of th
		*/
	//if(validPrevFame &&(currentFrame.pidControlMsgType == L_LANE || 
	//					currentFrame.pidControlMsgType == R_LANE ||	
	//					currentFrame.pidControlMsgType == UNKNOWN ||
	//					currentFrame.pidControlMsgType == HORIZONTAL))
	if(currentFrame.lanePatern == UNKNOWN || currentFrame.lanePatern == NOLANE){
		!CONT_DBG?:printf("Unknown Lane pattern\n");
		p2.x = w/2;
		p2.y = 0;
		p1.x = w/2;
		p1.y = h; // Go straight
		angleOffset = 0;
	}

	if(validPrevFame==1 && currentFrame.lanePatern == HORIZONTAL)
	{
		/* Only direction is stored for now
			Have to add sign and other stuffs. 
		*/
		if(previousFrame.nextDirection != NONE){
			printf("Entering Case.! Mode:");
			switch(previousFrame.nextDirection){
				case STRAIGHT:	// Maintain the path But invalid for this case
					// Choose based on angle(angle == y position)
					/*if(p2.y>p1.x){
						p2.x=lanes_org[horizLane].p1.x;
						p2.y=lanes_org[horizLane].p1.y;
					}
					else{
						p2.x=lanes_org[horizLane].p2.x;
						p2.y=lanes_org[horizLane].p2.y;	
					}*/
					p2.x = w/2;
					p2.y = 0;
					p1.x = w/2;
					p1.y = h;
					angleOffset = -90; // random case
					break;
				case LEFT:
					/*if(lanes_org[horizLane].p1.x<lanes_org[horizLane].p2.x){
						p2.x=lanes_org[horizLane].p1.x;
						p2.y=lanes_org[horizLane].p1.y;
					}else{
						p2.x=lanes_org[horizLane].p2.x;
						p2.y=lanes_org[horizLane].p2.y;
					}*/
					printf("LEFT.!\n");
					p2.x = w/2;
					p2.y = 0;
					p1.x = w/2;
					p1.y = h;
					angleOffset = 90; // Go right
					break;
				case RIGHT:
					/*if(lanes_org[horizLane].p1.x>lanes_org[horizLane].p2.x){
						p2.x=lanes_org[horizLane].p1.x;
						p2.y=lanes_org[horizLane].p1.y;
					}else{
						p2.x=lanes_org[horizLane].p2.x;
						p2.y=lanes_org[horizLane].p2.y;
					}*/
					printf("RIGHT.!\n");
					p2.x = w/2;
					p2.y = 0;
					p1.x = w/2;
					p1.y = h;
					angleOffset = -90; // Go left
					break;
				case BACK: break;
				case UTURN:
					printf("U-TURN.!\n");
					/*p2.x=(lanes_org[horizLane].p1.x+lanes_org[horizLane].p2.x)/2;
					p2.y=(lanes_org[horizLane].p1.y+lanes_org[horizLane].p1.y)/2;*/
					p2.x = w/2;
					p2.y = 0;
					p1.x = w/2;
					p1.y = h;
					angleOffset = 180;					
					break;
				case STOP:
					p2.x = w/2;
					p2.y=h;
					printf("Reached Stop sign.! Terminating control.\n");
					exit(0);
					break;
			}
			validResult = 1;
			// Reset the tracking details
			previousFrame.valid=0;
			previousFrame.nextDirection = NONE;
			currentFrame.nextDirection = NONE;
			currentFrame.lanePatern = NOLANE;
		}
		else{
			!CONT_DBG?:printf(" No valid previous pattern for Horizontal Lane. (Default turn)\n");
			// Set to left as default
			p2.x = w/2;
			p2.y = 0;
			p1.x = w/2;
			p1.y = h;
			angleOffset = 90;
			validResult = 1;
			previousFrame.valid=0;
		}
	}else if(currentFrame.lanePatern == HORIZONTAL){
		// Choose based on angle(angle == y position)
		/*if(p2.y>p1.x){
			p2.x=lanes_org[horizLane].p1.x;
			p2.y=lanes_org[horizLane].p1.y;
		}
		else{
			p2.x=lanes_org[horizLane].p2.x;
			p2.y=lanes_org[horizLane].p2.y;	
		}*/
		!CONT_DBG?:printf("Starting point is horizontal lane.!\n");
		p2.x = w/2;
		p2.y = 0;
		p1.x = w/2;
		p1.y = h;
		angleOffset = 90; // random case			
		validResult = 1;
	}
	//printf("Horizontal Lane- Critical Path.!\n" ); // Decide Contol based on previous Values	



	// Angle calculation
	int dx = p1.x-p2.x;
	int dy = p1.y-p2.y;
	!CONT_DBG?:printf("Angle: dx=%d dy=%d  w=%d, h=%d\n",dx,dy,w,h);
	float angle = atan2f(dy, dx) * 180/CV_PI;
	float error;
	error = angle-90;
	!CONT_DBG?:printf("\nNON_Correction: %f Porjection Angle: %f, angleOffset: %f\n",error,angle,angleOffset);
	// currentFram
	if(currentFrame.lanePatern == NORMAL)
		error = error-2*error;	// Convert to opposit direction since we are using mid-points
	else
		error = error+angleOffset;


	int length = 70;
	CvPoint P1_tmp;
	P1_tmp.x = (int)w/2;
	P1_tmp.y = h;
	CvPoint P2_tmp;
	
	P2_tmp.x = /*p2.x ;//*/(int)round(P1_tmp.x + length * cos((error+90) * CV_PI / 180.0));
	P2_tmp.y = /*p2.y ;//*/(int)round(P1_tmp.y - length * sin((error+90) * CV_PI / 180.0));
	!CONT_DBG?:printf("Correction Vector:%d*%d (%d,%d) (%d,%d) \n",w,h,P1_tmp.x,P1_tmp.y,P2_tmp.x,P2_tmp.y);

	line(im_roi, P1_tmp, P2_tmp, CV_RGB(200, 10, 10), 2, 8);
	sprintf(result,"%s","Correction");
	putText(im_roi, result, p1, FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255,255), 1, CV_AA);

	!CONT_DBG?:printf("Correction(anti-clock): %f Porjection Angle: %f, angleOffset: %f\n",error,angle,angleOffset);
	if(CONT_IMG)	showIm("Projection Lines",im_roi);
	sendCmd(error);
}



void findBorderPoints(std::vector<Lane> lanes_org, int l, CvPoint *lowPt,CvPoint *highPt, int w, int h ){
	//Upper boundary
	if(lanes_org[l].intersectUpper > w){
		highPt->x=w;
		highPt->y=lanes_org[l].m*w+lanes_org[l].c;
	}
	else if(lanes_org[l].intersectUpper <0){
		highPt->x=0;
		highPt->y= lanes_org[l].c;
	}
	else{
		highPt->x = (int)lanes_org[l].intersectUpper;
		highPt->y = 0;
	}

	//Lower boundary
	if(lanes_org[l].intersectLower >w){
		lowPt->x=w;
		lowPt->y=lanes_org[l].m*w+lanes_org[l].c;
	}
	else if(lanes_org[l].intersectLower <0){
		lowPt->x=0;
		lowPt->y= lanes_org[l].c;
	}
	else{
		lowPt->x = (int)lanes_org[l].intersectLower;
		lowPt->y = h;
	}
	!LINE_DBG?:printf("BorderPoints: (%d,%d) (%d,%d)\n",highPt->x,highPt->y, lowPt->x,lowPt->y);
}	

void laneFilter(std::vector<Lane> lanes_org, int w, int h,\
			int *leftLane, int *rightLane, int *horizLane ){
	// Init
	*leftLane = -1;
	*rightLane = -1;
	*horizLane = -1;

	int refPoint = w/2;
	Mat res_im = Mat::zeros( h, w, CV_8UC3 );

	int horLanePos = -1;
	int verLanePosL = -1;
	int verLanePosR = -1;
	// Tmp data
	std::vector<Lane> Hlanes;

	for	(int i=0; i<lanes_org.size(); i++) {
		//printf("Lane: %d, refPt:%d\n",i,refPoint);
		if(lanes_org[i].L_type == HORIZ_LINE){
			!LINEF_DBG?:printf(" Hor Lane dist: %f\n",(lanes_org[i].intersectLower+lanes_org[i].intersectUpper) );
			//printf("Type:Horiz  lowInterSect: %f upIntersect:%f \n",lanes_org[i].intersectLower,lanes_org[i].intersectUpper);
			if((lanes_org[i].intersectLower+lanes_org[i].intersectUpper)> HORIZ_LANE_MIN_DIST){  		// If distance satisfies the requirement
				printf("Type:Horiz  lowInterSect: %f upIntersect:%f \n",lanes_org[i].intersectLower,lanes_org[i].intersectUpper);
				//printf("Good lane..!\n");
				Hlanes.push_back(lanes_org[i]);
			}
			else printf("bad lane..%f !\n",(lanes_org[i].intersectLower+lanes_org[i].intersectUpper));
		}
		else{		// Vertical Lane -> Detect the lane which is closest to reference axis
			//printf("Type:Vertical  lowInterSect: %f upIntersect:%f \n",lanes_org[i].intersectLower,lanes_org[i].intersectUpper);
			if(lanes_org[i].intersectLower <= refPoint){  // Left Lane
				//printf("In1 :%d \n",verLanePosL );
				if(verLanePosL == -1){
					//printf("Validated1\n" );
					verLanePosL = i;
				}
				else if(lanes_org[verLanePosL].intersectLower<lanes_org[i].intersectLower){
					//printf("Validated2\n" );
					verLanePosL = i;
				}else{

				}
			}
			else{	// Right Lane
				//printf("In1 :%d \n",verLanePosR );
				if(verLanePosR == -1){
					//printf("Validated3\n" );
					verLanePosR = i;
				}
				else if(lanes_org[verLanePosR].intersectLower>lanes_org[i].intersectLower){
					verLanePosR = i;
					//printf("Validated4\n" );
				}
			}
		}
		//printf("Done\n\n\n" );
	}

	// Horizontal Lane detection
	// To do: For simplicity, only first horizontal lane is taken into account.! Need improvement on this.
	for	(int i=0; i<Hlanes.size(); i++) {
		!LINEF_DBG?:printf(" Horiz_id:%d\n",i);
		if(horLanePos == -1)
			horLanePos = i;
		else if(lanes_org[horLanePos].intersectLower<lanes_org[i].intersectLower){
			horLanePos = i;
		}
	}

	//Display
	!LINEF_DBG?:printf("Vertical Lane Left: %d , Right: %d     Horiz:%d\n",verLanePosL,verLanePosR,horLanePos);
	if(verLanePosL != -1){
		*leftLane=verLanePosL;
		line(res_im, lanes_org[verLanePosL].p1, lanes_org[verLanePosL].p2, CV_RGB(2*50, 50, 200), 2, 8);
		sprintf(result,"VerL");
		putText(res_im, result, lanes_org[verLanePosL].p1, FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255,255), 1, CV_AA);
	}
	if(verLanePosR!=-1){
		*rightLane=verLanePosR;
		line(res_im, lanes_org[verLanePosR].p1, lanes_org[verLanePosR].p2, CV_RGB(3*50, 50, 200), 2, 8);
		sprintf(result,"VerR");
		putText(res_im, result, lanes_org[verLanePosR].p1, FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255,255), 1, CV_AA);
	}
	if(horLanePos!=-1){
		*horizLane = horLanePos;
		line(res_im, lanes_org[horLanePos].p1, lanes_org[horLanePos].p2, CV_RGB(4*50, 50, 200), 2, 8);
		sprintf(result,"Hor");
		putText(res_im, result, lanes_org[horLanePos].p1, FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255,255), 1, CV_AA);
	}
	if(LINEF_IMG) showIm("Lane filter Output",res_im);
}


Mat preProcess(Mat image){
	Mat im_roi;
	//showIm("DBG:Org",image);
	//resize(image, image, Size(800, 600), 0, 0, INTER_CUBIC); 
	//vk-pyrDown(image, image, Size(image.cols/2, image.rows/2));
	//pyrDown(image, image, Size(image.cols/2, image.rows/2));
	//showIm("Original Image",image);
	//showIm("DBG:Pry",image);

	// Configure
	int size_x = image.cols;
	int size_y = image.rows;
	int size_z = 3;
	int ROI_w = size_x;
	int ROI_h = size_y/2;
	int ROI_x = 0;
	int ROI_y = size_y-ROI_h;
	ROI =  cvRect(ROI_x,ROI_y,ROI_w,ROI_h);
	
	// ROI processing (crop+gray_scale)
	Mat im_gray, im_hist, im_display1,im_display2;
	cvtColor(image, im_gray, CV_BGR2GRAY);
	//showIm("DBG:im_gray",im_gray);
	//equalizeHist( im_gray, im_gray );
	//equalizeHist( im_gray, im_gray );				// vk-hist is not used
	im_roi = im_gray(ROI).clone();
	im_display1 = image(ROI).clone();
	im_display2 = image(ROI).clone();
	//showIm("Image-ROI",im_roi);
	
	// Filtering
	medianBlur  (im_roi, im_roi, 5 );
	GaussianBlur(im_roi, im_roi,Size( 3, 3 ), 0, 0 );

	// /showIm("DBG:GaussianBlur",im_roi);
	//blur(im_roi,im_roi,Size( 3, 3 ),Point(-1,-1) );
	
		
	// DBG Code -remove VK
	Mat th_im;
	//threshold(im_roi,th_im, 60, 255, THRESH_BINARY_INV);
	//adaptiveThreshold(im_roi,th_im, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV,11,5);
	adaptiveThreshold(im_roi,th_im, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV,11,5);//15 2
	//GaussianBlur(im_roi,im_roi,Size( 3, 3 ), 0, 0 );
	//showIm("DBG:Thresh",th_im);
	int erosion_size = 1;
    int erosion_type = MORPH_RECT;
	Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );


	dilate( th_im, th_im, element );
	im_roi = th_im;
	//waitKey(0);
	return im_roi;
}
// Find lanes
void findLane(vector<Vec4i> *lines, int row, int col,std::vector<Lane> *lane){
	CvPoint p1 = CvPoint();
	CvPoint p2 = CvPoint();
	Mat res_im = Mat::zeros( row,col, CV_8UC3 );

	for	(int i=0; i<lines->size(); i++) {
		p1.x = lines->at(i)[0];
		p1.y = lines->at(i)[1];
		p2.x  = lines->at(i)[2];
		p2.y = lines->at(i)[3];

		int dx = p2.x - p1.x;
		int dy = p2.y - p1.y;

		// Reserved for future use
		float angle = atan2f(dy, dx) * 180/CV_PI; 		// Arc tangent (hough angle)
		dx = (dx == 0) ? 1 : dx; // prevent DIV/0! 
		float m = dy/(float)dx;
		float c = p1.y - m*p1.x;
		m = (m==0)?1:m;
		float lineDist = sqrt(dx*dx+dy*dy);

		float intersectUpper = -1;
		float intersectLower = -1;
		enum LineType L_type;

		// Intersect at image border
		if(fabs(angle)>=LINE_SEP_ANGLE){		// Vertical Lines
			intersectUpper = -c/m;
			intersectLower = (row-c)/m;
			L_type = VERTICAL_LINE;
		}
		else{									// Horizontal Lines
			intersectLower = c;
			intersectUpper = m*col+c;
			L_type = HORIZ_LINE;
		}

		if(fabs(angle) <= 90-POST_ANGLE || fabs(angle) >= 90+POST_ANGLE)
			lane->push_back(Lane(p1, p2, angle, m, c, intersectLower, intersectUpper, L_type,lineDist,1));
		line(res_im, p1, p2, CV_RGB(i*50, 50, 200), 2, 8);
		//printf("Lane-%d: (%d,%d) (%d,%d) \n",i,p1.x,p1.y,p2.x,p2.y);
		!LINE_DBG?:printf("Lane-%d: (%d,%d) (%d,%d) -> (Low:%f Up:%f) Enum:%d Angle: %f\n",\
			i,p1.x,p1.y,p2.x,p2.y,intersectLower,intersectUpper,L_type,angle);
		sprintf(result,"%d",i);
		putText(res_im, result, p1, FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255,255), 1, CV_AA);
	}	
	if(LINE_IMG) showIm("Image-Hough Lines",res_im);
}

// Util
void init(){
	previousFrame.valid=0;
	currentFrame.valid =0;
	uart0_filestream = openPort();
    setupSerial(uart0_filestream);
    // Sign info
    detectedSignInfo.direction = NONE;
    detectedSignInfo.area=0;
    detectedSignInfo.confidanceCount=0;
    for(int i=0;i<SIGN_TEMPORAL_WIN;i++){
    	signWindow[i].direction = NONE;
		signWindow[i].area=0;
		signWindow[i].confidanceCount=0;
    }
#if EN_PWR_LOG
    logFile = startLog(); 
#endif
}

// Display image
void showIm(String title, Mat im){
#if IMG_ON
	namedWindow(title, WINDOW_AUTOSIZE );
    imshow(title, im);
#endif
}

float dist(CvPoint p1, CvPoint p2){
	return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}
CvPoint intersection(CvPoint o1_t, CvPoint p1_t, CvPoint o2_t, CvPoint p2_t)
{
	Point2f o1,p1,o2,p2,r;
	CvPoint r_t;
	o1.x=o1_t.x;  o1.y=o1_t.y;
	p1.x=p1_t.x;  p1.y=p1_t.y;
	o2.x=o2_t.x;  o2.y=o2_t.y;
	p2.x=p2_t.x;  p2.y=p2_t.y;
	
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8){
         //return false;
    	printf("ERROR Ln.1012 main.c: Unreachable case.!\n");
    	r_t.x=0;r_t.y=0;
    	return r_t;
    }

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    r_t.x=r.x;  r_t.y=r.y;
    return r_t;
}

void dir2str(DirectionVector dir, char *var){
	switch(dir){
    	case(STRAIGHT): strcpy(var, "STRAIGHT")	;break;
		case(LEFT): 	strcpy(var, "LEFT"	)	;break;
		case(RIGHT): 	strcpy(var, "RIGHT"	)	;break;
		case(BACK): 	strcpy(var, "BACK"	)	;break;
		case(UTURN): 	strcpy(var, "UTURN"	)	;break;
		case(STOP): 	strcpy(var, "STOP"	)	;break;
		case(NONE): 	strcpy(var, "NONE"	)	;break;
		default: 		strcpy(var, "unknown case");
    }
}
void conf2str(Confidence conf, char *var){
	switch(conf){
    	case(HIGH): 	strcpy(var, "HIGH"	)	;break;
		case(MEDIUM): 	strcpy(var, "MEDIUM")	;break;
		case(LOW): 		strcpy(var, "LOW"	)	;break;
		case(POOR): 	strcpy(var, "POOR"	)	;break;
		default: 		strcpy(var, "unknown case");
    }
}
void pid2str(PIDRotation rot, char *var){
	switch(rot){
    	case(TRANSLATE): 	strcpy(var, "TRANSLATE"	)	;break;
		case(ROTATE): 	strcpy(var, "ROTATE")	;break;
		default: 		strcpy(var, "unknown case");
    }
}
void lanePat2str(LanePattern pat, char *var){
	switch(pat){
		case(NORMAL): strcpy(var,"NORMAL");break;
		case(HORIZONTAL): strcpy(var,"HORIZONTAL");break;
		case(L_LANE): strcpy(var,"L_LANE");break;
		case(R_LANE): strcpy(var,"R_LANE");break;
		case(CORNER): strcpy(var,"CORNER");break;
		case(LINE_BREAK): strcpy(var,"LINE_BREAK");break;
		case(UNKNOWN): strcpy(var,"UNKNOWN");break;
		case(ERROR): strcpy(var,"ERROR");break;
		case(NOLANE): strcpy(var,"NOLANE");break;
		default: 		strcpy(var, "unknown case");
    }
}
void printControlInfo(char *var, ControlInfo pt){
	char var1[50];dir2str(pt.trafficSign,var1);
	char var2[50];dir2str(pt.direction,var2);
	char var3[50];conf2str(pt.dirCounter,var3);
	char var4[50];conf2str(pt.signCounter,var4);
	char var5[50];lanePat2str(pt.lanePatern,var5);
	char var6[50];dir2str(pt.nextDirection,var6);
	char var7[50];pid2str(pt.pidControlMsgType,var7);

	printf("%s ControlInfo:\n",var);
	printf("\t direction = %s\n",var2);
	printf("\t angle = %d\n",pt.angle);
	printf("\t trafficSign = %s\n",var1);
	printf("\t dirCounter = %s\n"	,var3);			// Lane confidence
	printf("\t signCounter = %s\n"	,var4);		// Sign Confidence
	printf("\t valid = %d\n", pt.valid);
	printf("\t lanePatern = %s\n", var5);
	printf("\t nextDirection = %s\n", var6); 
	printf("\t pidControlMsgType = %s\n", var7);
}
