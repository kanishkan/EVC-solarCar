#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "config.h"
#include "serialCar.h" 

using namespace cv;
using namespace std;
Rect ROI;

// Input Configuration
#define VIDEO_INPUT 0
#define VIDEO_FILE 1
#define ADVANCE_CASE 0
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

float dist(CvPoint p1, CvPoint p2);
CvPoint intersection(CvPoint o1, CvPoint p1, CvPoint o2, CvPoint p2);
struct ControlInfo previousFrame;
struct ControlInfo currentFrame;

int validResult = 0;

void sendCmd(float angle){
	if(validResult == 0)
		return;
	if((int)fabs(angle)>99)
		sprintf(command,"%dT%s",(int)fabs(angle), angle<0?"-":"+");
	else
		sprintf(command,"0%dT%s",(int)fabs(angle), angle<0?"-":"+");
	printf("Serial Data: %s\n",command);
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
while(1){
    waitKey();
    cap >> image;

#elif VIDEO_FILE
	VideoCapture cap("video_new.h264");
	if(!cap.isOpened()){  // check if we succeeded
     	printf("Unable to open video file.!\n");
        return -1;
    }
	while(1)
	{
		waitKey(5000);
		 Mat rawImg;
		 for(int fRate=0;fRate<10;fRate++)
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
    // Pre-process
	Mat im_roi,im_edge;
	im_roi = preProcess(image);

	// Edge-detection
	Canny(im_roi, im_edge, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);
	showIm("Image-Edges",im_edge);

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

	// Command PID
	/* Communication protocol driver
	*/
}
	waitKey(0);
}

CvPoint rightLaneLowPt, rightLaneHighPt;
CvPoint leftLaneLowPt, leftLaneHighPt;

void updateTrackingInfo(){
	if(currentFrame.lanePatern==UNKNOWN || currentFrame.lanePatern==NOLANE ){
		// No lanes found; leave the previous pattern previous pattern and update its temportal location.
		previousFrame.dirCounter = LOW;
	}
	else{
		if(currentFrame.lanePatern==HORIZONTAL || currentFrame.lanePatern==CORNER || currentFrame.lanePatern==ERROR){ // Start as new since rotation is done
			previousFrame.valid = 0;
		}
		else{ // Valid pattern; update values
			previousFrame.direction = currentFrame.direction;         
			previousFrame.angle = currentFrame.angle;
			previousFrame.trafficSign = currentFrame.trafficSign;
			previousFrame.dirCounter = HIGH;
			previousFrame.signCounter = currentFrame.signCounter; // Not used yet
			previousFrame.valid = 1;
			previousFrame.lanePatern = currentFrame.lanePatern;
			previousFrame.nextDirection = currentFrame.direction ;
			previousFrame.pidControlMsgType = currentFrame.pidControlMsgType;
		}
	}
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

	// Calculate boarder Crossing points
	if(leftLane!=-1){
		printf("LeftLane ");
		findBorderPoints(lanes_org, leftLane, &leftLaneLowPt, &leftLaneHighPt, w,h );
	}
	if(rightLane!=-1){
		printf("RightLane ");
		findBorderPoints(lanes_org, rightLane, &rightLaneLowPt,&rightLaneHighPt,w,h);
	}
	// Find mid points
	int lowerMid,upperMid,horizMidx,horizMidy;
	int verticalLaneDist = 0;
	if(leftLane!=-1) 
		lowerMid= (rightLaneLowPt.x +leftLaneLowPt.x)/2;
	if(rightLane!=-1)
		upperMid = (rightLaneHighPt.x +leftLaneHighPt.x)/2;
	if(horizLane != -1){
		horizMidx = (lanes_org[horizLane].p1.x+lanes_org[horizLane].p2.x)/2;
		horizMidy = (lanes_org[horizLane].p1.y+lanes_org[horizLane].p2.y)/2;
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
		if(distToHorizLane>HORIZ_LANE_MIN_DIST || horizMidy<=HORIZ_LANE_LEFT_REJECTION || horizMidy>=HORIZ_LANE_RIGHT_REJECTION)			// Distance is high | Left border |Right border
			discardHorizLane = 1;
	}
	printf(" Loop Start: horLane =%d, discard:%d\n",horizLane,discardHorizLane);
	if(horizLane== -1 || discardHorizLane ==1){		// Only vertical Lanes
		if(leftLane!=-1 && rightLane!=-1){		// Two Lanes
			if(leftLaneLowPt.x < rightLaneLowPt.x && leftLaneHighPt.x < rightLaneHighPt.x){ // Lane does not cross			
				verticalLaneDist = dist(rightLaneLowPt,leftLaneLowPt);
				if(verticalLaneDist<VERT_LANE_SEPERATION){
					// Lanes are too close # Not exected
					printf("Vertical Lanes are too close.\n");
					currentFrame.lanePatern = UNKNOWN;
				}else{
					printf("Vertical Lanes : Strong values\n");
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
				printf("Predicited Lanes crossing.!\n");
				interSectPt = intersection(leftLaneLowPt, leftLaneHighPt, rightLaneLowPt, rightLaneHighPt);
				if(interSectPt.x==0 && interSectPt.y==0){
					printf("Could not find intersect point.!\n");
					currentFrame.lanePatern = UNKNOWN;
				}
				else{
					printf("Vertical Line intersect point: (%d,%d).!\n",interSectPt.x,interSectPt.y);	
					p1.x = lowerMid;
					p1.y = h;
					p2.x = interSectPt.x;
					p2.y = interSectPt.y;
					currentFrame.lanePatern = CORNER;
					validResult = 1;
				}
			}
		}else{ // One Vertical Lane
			p1.x = w/2;
			p1.y = h;
			if(leftLane!=-1){	// Left Lane alone
				printf("Signle Left Lane- Angle:%f\n",lanes_org[leftLane].angle);
				//angleOffset = 45-lanes_org[leftLane].angle;
				angleOffset = lanes_org[leftLane].angle+90;
				//p2.x = leftLaneHighPt;
				//p2.y = 0;
				currentFrame.lanePatern = L_LANE;
				validResult = 1;
			}
			else if(rightLane!=-1){
				printf("Signle Right Lane- Angle:%f\n",lanes_org[rightLane].angle);
				//p2.x = rightLaneHighPt;
				//p2.y = 0;
				//angleOffset = lanes_org[rightLane].angle-45;
				angleOffset = lanes_org[rightLane].angle+90;
				currentFrame.lanePatern = R_LANE;
				validResult = 1;
			}
			else{	// No vertical Lane
				printf("Horizontal Lane is far from current position.\n" );
				currentFrame.lanePatern = NOLANE;
			}
		}
	}else if(horizLane != -1){											// Horizontal and Verival Lane
		// Distance to horizontal Lane = horizMidy
		distToHorizLane = h-horizMidy;	
		if(leftLane!=-1 && rightLane!=-1){			// Two vertical and one horizontal
			//Set intersect point to mid of two vertical lanes
			printf("Horizontal Lane + 2 vertical lane.\n" );
			CvPoint leftLInt = intersection(leftLaneLowPt, leftLaneHighPt,lanes_org[horizLane].p1,lanes_org[horizLane].p2 );
			CvPoint rightLInt = intersection(rightLaneLowPt, rightLaneHighPt,lanes_org[horizLane].p1,lanes_org[horizLane].p2);
			p2.x = (leftLInt.x+rightLInt.x)/2;
			p2.y = (leftLInt.y+rightLInt.y)/2;
			validResult = 1;
		}else{	
			if(leftLane!=-1){	// Left Lane alone
				printf("Horizontal + Signle Left Lane- Angle:%f\n",lanes_org[rightLane].angle);
				CvPoint leftLInt = intersection(leftLaneLowPt, leftLaneHighPt,lanes_org[horizLane].p1,lanes_org[horizLane].p2 );
				printf("Horizontal+Left: Intersect:(%d,%d)\n",leftLInt.x,leftLInt.y);
				if(leftLInt.y<= h/2){ // Avoid sharp ende on close
					printf("Not a sharp edge.!\n");
					p2.x = (leftLInt.x +horizMidx )/2;
					p2.y = (leftLInt.y +horizMidy )/2;
				}else{	// 
					printf("Sharp edge.! Avoid maximum value.\n");
					if(lanes_org[horizLane].p1.x>lanes_org[horizLane].p2.x){
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
				printf("Horizontal + Signle Right Lane- Angle:%f\n",lanes_org[rightLane].angle);
				CvPoint rightLInt = intersection(leftLaneLowPt, leftLaneHighPt,lanes_org[horizLane].p1,lanes_org[horizLane].p2 );
				printf("Horizontal+Right: Intersect:(%d,%d)\n",rightLInt.x,rightLInt.y);
				if(rightLInt.y<= h/2){ // Avoid sharp ende on close
					printf("Not a sharp edge.!\n");
					p2.x = (rightLInt.x +horizMidx )/2;
					p2.y = (rightLInt.y +horizMidy )/2;
				}else{	// 
					printf("Sharp edge.! Avoid maximum value.\n");
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
				printf("Only Horizontal Lanes.! Previous frame: %s \n", (validPrevFame==1)?"Valid(Control:Prev.frame)":"No(Control:Random)"); 
				currentFrame.lanePatern = HORIZONTAL;
			}
		}	
	}
	else{
		printf(" No Lanes.. :(\n");
		currentFrame.lanePatern = NOLANE;
	}

	
	

	//line(im_roi, p1, p2, CV_RGB(200, 10, 10), 2, 8);
	//printf("Projection Line: (%d,%d) (%d,%d) \n",p1.x,p1.y,p2.x,p2.y);
	//sprintf(result,"%s","Projected Line");
	//putText(im_roi, result, p1, FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255,255), 1, CV_AA);
	// Load last frame info:


	if(currentFrame.lanePatern == CORNER){	// Do 135 degree turn
		angleOffset = 135;
		printf("Corner Case-> Adding roation.!");
	}	

//	else if(currentFrame.lanePatern == HORIZONTAL){	
	else if(currentFrame.lanePatern == HORIZONTAL && distToHorizLane<=HORIZ_LANE_CRITICAL_DIST)		// Immediate response
		currentFrame.pidControlMsgType = ROTATE;
		/* Not checking for length of th
		*/
	//if(validPrevFame &&(currentFrame.pidControlMsgType == L_LANE || 
	//					currentFrame.pidControlMsgType == R_LANE ||	
	//					currentFrame.pidControlMsgType == UNKNOWN ||
	//					currentFrame.pidControlMsgType == HORIZONTAL))
	if(validPrevFame==1 && currentFrame.lanePatern ==HORIZONTAL)
	{
		/* Only direction is stored for now
			Have to add sign and other stuffs. 
		*/
		if(previousFrame.nextDirection == !NONE){
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
					p2.x = w/2;
					p2.y = 0;
					p1.x = w/2;
					p1.y = h;
					angleOffset = 90; // Go left
					break;
				case BACK: break;
				case UTURN:
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
					break;
			}
		}
		else{
			printf(" Initial Condition is Horizontal Lane. (Default turn)\n");
			// Set to left as default
			p2.x = w/2;
			p2.y = 0;
			p1.x = w/2;
			p1.y = h;
			angleOffset = 90;
		}
	}else if(currentFrame.lanePatern ==HORIZONTAL){
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
		angleOffset = 90; // random case			
		validResult = 1;
	}
	//printf("Horizontal Lane- Critical Path.!\n" ); // Decide Contol based on previous Values	
	if(currentFrame.lanePatern == UNKNOWN || currentFrame.lanePatern == NOLANE){
		printf("Unknown Lane pattern\n");
		p2.x = w/2;
		p2.y = 0;
		p1.x = w/2;
		p1.y = h; // Go straight
		angleOffset = 0;
	}


	// Angle calculation
	int dx = p1.x-p2.x;
	int dy = p1.y-p2.y;
	float angle = atan2f(dy, dx) * 180/CV_PI;
	float error;
	error = angle-90;
	printf("\nNON_Correction: %f Porjection Angle: %f, angleOffset: %f\n",error,angle,angleOffset);
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
	printf("Correction Vector:%d*%d (%d,%d) (%d,%d) \n",w,h,P1_tmp.x,P1_tmp.y,P2_tmp.x,P2_tmp.y);

	line(im_roi, P1_tmp, P2_tmp, CV_RGB(200, 10, 10), 2, 8);
	sprintf(result,"%s","Correction");
	putText(im_roi, result, p1, FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255,255), 1, CV_AA);

	printf("Correction(anti-clock): %f Porjection Angle: %f, angleOffset: %f\n",error,angle,angleOffset);
	showIm("Projection Lines",im_roi);
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
	printf("BorderPoints: (%d,%d) (%d,%d)\n",highPt->x,highPt->y, lowPt->x,lowPt->y);
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
			printf(" Hor Lane dist: %f\n",(lanes_org[i].intersectLower+lanes_org[i].intersectUpper)/2 );
			if((lanes_org[i].intersectLower+lanes_org[i].intersectUpper)> 2*HORIZ_LANE_MIN_DIST){  		// If distance satisfies the requirement
				//printf("Type:Horiz  lowInterSect: %f upIntersect:%f \n",lanes_org[i].intersectLower,lanes_org[i].intersectUpper);
				Hlanes.push_back(lanes_org[i]);
			}
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
		printf(" Horiz_id:%d\n",i);
		if(horLanePos == -1)
			horLanePos = i;
		else if(lanes_org[horLanePos].intersectLower<lanes_org[i].intersectLower){
			horLanePos = i;
		}
	}

	//Display
	printf("Vertical Lane Left: %d , Right: %d     Horiz:%d\n",verLanePosL,verLanePosR,horLanePos);
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
	showIm("Lane filter Output",res_im);
}

Mat preProcess(Mat image){
	Mat im_roi;
	//resize(image, image, Size(800, 600), 0, 0, INTER_CUBIC); 
	pyrDown(image, image, Size(image.cols/2, image.rows/2));
	//pyrDown(image, image, Size(image.cols/2, image.rows/2));
	//showIm("Original Image",image);
	
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
	//equalizeHist( im_gray, im_gray );				// vk-hist is not used
	im_roi = im_gray(ROI).clone();
	im_display1 = image(ROI).clone();
	im_display2 = image(ROI).clone();
	//showIm("Image-ROI",im_roi);
	
	// Filtering
	GaussianBlur(im_roi,im_roi,Size( 3, 3 ), 0, 0 );
	//blur(im_roi,im_roi,Size( 3, 3 ),Point(-1,-1) );
	// medianBlur ( im_roi, im_roi, 5 );
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

		lane->push_back(Lane(p1, p2, angle, m, c, intersectLower, intersectUpper, L_type,lineDist,1));
		line(res_im, p1, p2, CV_RGB(i*50, 50, 200), 2, 8);
		//printf("Lane-%d: (%d,%d) (%d,%d) \n",i,p1.x,p1.y,p2.x,p2.y);
		printf("Lane-%d: (%d,%d) (%d,%d) -> (Low:%f Up:%f) Enum:%d Angle: %f\n",\
			i,p1.x,p1.y,p2.x,p2.y,intersectLower,intersectUpper,L_type,angle);
		sprintf(result,"%d",i);
		putText(res_im, result, p1, FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255,255), 1, CV_AA);
	}	
	showIm("Image-Hough Lines",res_im);
}

// Util
void init(){
	previousFrame.valid=0;
	currentFrame.valid =0;
	uart0_filestream = openPort();
    setupSerial(uart0_filestream);
}

// Display image
void showIm(String title, Mat im){
	namedWindow(title, WINDOW_AUTOSIZE );
    imshow(title, im);
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
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    r_t.x=r.x;  r_t.y=r.y;
    return r_t;
}
