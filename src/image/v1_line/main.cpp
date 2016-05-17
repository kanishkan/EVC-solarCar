#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "utils.h"

using namespace cv;
using namespace std;

// Input
#define VIDEO_INPUT 0

//Input image config
int size_x;
int size_y;
int size_z;

// ROI Configuration
int ROI_w;
int ROI_h;
int ROI_x;
int ROI_y;
Rect ROI ;

// Parameters
#define MAX_LOST_FRAMES 30
#define K_VARY_FACTOR 0.2f
#define B_VARY_FACTOR 20

enum{
	CANNY_MIN_TRESHOLD = 300,	  // edge detector minimum hysteresis threshold
	CANNY_MAX_TRESHOLD = 400, // edge detector maximum hysteresis threshold
	
	HOUGH_TRESHOLD = 20,		// line approval vote threshold
	HOUGH_MIN_LINE_LENGTH = 100,	// remove lines shorter than this treshold
	HOUGH_MAX_LINE_GAP = 50,   // join lines to one with smaller than this gaps

	SCAN_STEP = 5,			  // in pixels
	LINE_REJECT_DEGREES = 10, // in degrees
    BW_TRESHOLD = 250,		  // edge response strength to recognize for 'WHITE'
    BORDERX = 10,			  // px, skip this much from left & right borders
	MAX_RESPONSE_DIST = 5,	  // px
	
};

// Supporting function
void findLane(std::vector<Vec4i> *lines, Mat edge, Mat original);


void showIm(String title, Mat im){
	namedWindow(title, WINDOW_AUTOSIZE );
    imshow(title, im);
}

struct Lane{
	Lane(){}
	Lane(CvPoint p1l, CvPoint p2l, float ang, float ml, float cl): p1(p1l),p2(p2l),angle(ang),m(ml),c(cl){}
	CvPoint p1;
	CvPoint p2;
	float angle, m, c;
};

struct Status {
	Status():reset(true),lost(0){}
	ExpMovingAverage m, c;
	bool reset;
	int lost;
};

struct Detection{
	CvPoint horizUpP1,horizUpP2,horizLowP1, horizLowP2;			 // Horizontal Hough point
	CvPoint verLP1,verLP2,verRP1, verRP2;						 // Vertical Hough points
	float m_leftVar, c_leftVar, m_rightVar, c_rightVar;			 // Verical hough Values
	float m_upperVar, c_upperVar, m_lowerVar, c_lowerVar;		 // Horizontal hough Values
	CvPoint hor_upper_p1,hor_upper_p2,hor_lower_p1,hor_lower_p2; // Horoizontal Projection points
	CvPoint ver_left_p1,ver_left_p2,ver_right_p1,ver_right_p2;	 // Vertical Projection points
	int hor_line_count,ver_line_count;		
	bool horixLowValid,horixUpValid,verleftValid,verRightValid;					 
};

Status laneR, laneL;
Status H_upper, H_lower;
Detection laneObj;



// Final lines


void FindResponses(Mat img, int startX, int endX, int y, std::vector<int>& list)
{
    // scans for single response: /^\_

	const int row = y * img.cols * img.channels();
	unsigned char* ptr = (unsigned char*)img.data;

    int step = (endX < startX) ? -1: 1;
    int range = (endX > startX) ? endX-startX+1 : startX-endX+1;

    for(int x = startX; range>0; x += step, range--)
    {
        if(ptr[row + x] <= BW_TRESHOLD) continue; // skip black: loop until white pixels show up

        // first response found
        int idx = x + step;

        // skip same response(white) pixels
        while(range > 0 && ptr[row+idx] > BW_TRESHOLD){
            idx += step;
            range--;
        }

		// reached black again
        if(ptr[row+idx] <= BW_TRESHOLD) {
            list.push_back(x);
        }

        x = idx; // begin from new pos
    }
}

void processSide(std::vector<Lane> lanes, Mat edges, bool right, bool horizLane) {

	Status* side;
	if(!horizLane)
		side = right ? &laneR : &laneL;
	else
		side = right ? &H_lower : &H_upper;
	// response search
	int w = edges.cols;
	int h = edges.rows;
	const int BEGINY = 0;
	const int ENDY = h-1;
	const int ENDX = right ? (w-BORDERX) : BORDERX;
	int midx = w/2;
	int midy = edges.rows/2;
	unsigned char* ptr = (unsigned char*)edges.data;

	// show responses
	int* votes = new int[lanes.size()];
	for(int i=0; i<lanes.size(); i++) votes[i++] = 0;

	for(int y=ENDY; y>=BEGINY; y-=SCAN_STEP) {
		std::vector<int> rsp;
		FindResponses(edges, midx, ENDX, y, rsp);

		if (rsp.size() > 0) {
			int response_x = rsp[0]; // use first reponse (closest to screen center)

			float dmin = 9999999;
			float xmin = 9999999;
			int match = -1;
			for (int j=0; j<lanes.size(); j++) {
				// compute response point distance to current line
				float d = dist2line(
						cvPoint2D32f(lanes[j].p1.x, lanes[j].p1.y), 
						cvPoint2D32f(lanes[j].p2.x, lanes[j].p2.y), 
						cvPoint2D32f(response_x, y));

				// point on line at current y line
				int xline = (y - lanes[j].c) / lanes[j].m;
				int dist_mid = abs(midx - xline); // distance to midpoint

				// pick the best closest match to line & to screen center
				if (match == -1 || (d <= dmin && dist_mid < xmin)) {
					dmin = d;
					match = j;
					xmin = dist_mid;
					break;
				}
			}

			// vote for each line
			if (match != -1) {
				votes[match] += 1;
			}
		}
	}

	int bestMatch = -1;
	int mini = 9999999;
	for (int i=0; i<lanes.size(); i++) {
		int xline = (midy - lanes[i].c) / lanes[i].m;
		int dist = abs(midx - xline); // distance to midpoint

		if (bestMatch == -1 || (votes[i] > votes[bestMatch] && dist < mini)) {
			bestMatch = i;
			mini = dist;
		}
	}

	if (bestMatch != -1) {
		Lane* best = &lanes[bestMatch];
		float m_diff = fabs(best->m - side->m.get());
		float c_diff = fabs(best->c - side->c.get());

		bool update_ok = (m_diff <= K_VARY_FACTOR && c_diff <= B_VARY_FACTOR) || side->reset;

		if(right){	// vk
			if(!horizLane){
				laneObj.m_rightVar = m_diff;
				laneObj.c_rightVar = c_diff;
				laneObj.verRP1 = lanes[bestMatch].p1;
				laneObj.verRP2 = lanes[bestMatch].p2;
				laneObj.verRightValid = true;
			}
			else{
				laneObj.m_lowerVar = m_diff;
				laneObj.c_lowerVar = c_diff;
				laneObj.horizLowP1 = cvPoint(lanes[bestMatch].p1.y,lanes[bestMatch].p1.x);
				laneObj.horizLowP2 = cvPoint(lanes[bestMatch].p2.y,lanes[bestMatch].p2.x);
				laneObj.horixLowValid = true;
			}			
		}
		else{
			if(!horizLane){
				laneObj.m_leftVar = m_diff;
				laneObj.c_leftVar = c_diff;
				laneObj.verLP1 = lanes[bestMatch].p1;
				laneObj.verLP2 = lanes[bestMatch].p2;
				laneObj.verleftValid = true;
			}
			else{
				laneObj.m_upperVar = m_diff; 
				laneObj.c_upperVar = c_diff;
				laneObj.horizUpP1 = cvPoint(lanes[bestMatch].p1.y,lanes[bestMatch].p1.x);
				laneObj.horizUpP2 = cvPoint(lanes[bestMatch].p2.y,lanes[bestMatch].p2.x);
				laneObj.horixUpValid = true;
			}
		}

		if(!horizLane)
			printf("side: %s, m vary: %.4f, c vary: %.4f, lost: %s\n", 
				(right?"RIGHT":"LEFT"), m_diff, c_diff, (update_ok?"no":"yes"));
		else
			printf("side: %s, m vary: %.4f, c vary: %.4f, lost: %s\n", 
				(right?"LOWER":"UPPER"), m_diff, c_diff, (update_ok?"no":"yes"));

		if (update_ok) {
			// update is in valid bounds
			side->m.add(best->m);
			side->c.add(best->c);
			side->reset = false;
			side->lost = 0;
		} else {
			// can't update, lanes flicker periodically, start counter for partial reset!
			side->lost++;
			if (side->lost >= MAX_LOST_FRAMES && !side->reset) {
				side->reset = true;
			}
		}

	} else {
		printf("no lanes detected - lane tracking lost! counter increased\n");
		side->lost++;
		if (side->lost >= MAX_LOST_FRAMES && !side->reset) {
			// do full reset when lost for more than N frames
			side->reset = true;
			side->m.clear();
			side->c.clear();
		}
		if(right)
			if(!horizLane)
				laneObj.verRightValid = false;
			else
				laneObj.horixLowValid = false;
		else
			if(!horizLane)
				laneObj.verleftValid  = false;
			else
				laneObj.horixUpValid  = false;
	}

	delete[] votes;
}

void findLane(vector<Vec4i> *lines, Mat edge, Mat original, bool horizLane){
	// classify lines to left/right side
	std::vector<Lane> left, right;

	Mat lane_image = original.clone();
	printf("Number of lines:%d \n",(int)lines->size());
	for(int i = 0; i < lines->size(); i++ )
    {
    	int dx = lines->at(i)[2] - lines->at(i)[0];
		int dy = lines->at(i)[3] - lines->at(i)[1];
		float angle = atan2f(dy, dx) * 180/CV_PI; 		// Arc tangent (hough angle)
		printf("line %d:(%d,%d) and (%d,%d) \n",i,lines->at(i)[0],lines->at(i)[1],lines->at(i)[2],lines->at(i)[3]);

		if (fabs(angle) <= LINE_REJECT_DEGREES) { // reject near horizontal lines
			continue;
		}

		// line parameters: y = mx + c;
		dx = (dx == 0) ? 1 : dx; // prevent DIV/0! 
		float m = dy/(float)dx;
		float c = lines->at(i)[1] - m*lines->at(i)[0];

		int midx = (lines->at(i)[0] + lines->at(i)[2]) / 2;
		CvPoint *p = new CvPoint[2];
		p[0].x = lines->at(i)[0];
		p[0].y = lines->at(i)[1];
		p[1].x = lines->at(i)[2];
		p[1].y = lines->at(i)[3];

		if (midx < original.cols/2) {
			left.push_back(Lane(p[0], p[1], angle, m, c));
		} else if (midx > original.cols/2) {
			right.push_back(Lane(p[0], p[1], angle, m, c));
		}
    }
    printf("Number of Left lanes:%d \n",(int)left.size());
    printf("Number of Right lanes:%d \n",(int)right.size());

    // show Hough lines
	for	(int i=0; i<right.size(); i++) {
		line(original, right[i].p1, right[i].p2, CV_RGB(i*5, i*20, 255), 2, 8);
		printf("Right lanes(%d): (%d,%d) (%d,%d) \n",i,right[i].p1.x,right[i].p1.y,right[i].p2.x,right[i].p2.y);
	}

	for	(int i=0; i<left.size(); i++) {
		line(original, left[i].p1, left[i].p2, CV_RGB(255, 0, i*20), 2, 8);
		printf("Left lanes(%d): (%d,%d) (%d,%d) \n",i,left[i].p1.x,left[i].p1.y,left[i].p2.x,left[i].p2.y);
	}
	showIm("Hough-Lines",original);

	processSide(left, edge, false, horizLane);
	processSide(right, edge, true, horizLane);

	// show computed lanes
	int x = lane_image.cols * 0.55f;
	int x2 = lane_image.cols;
	if(!horizLane){
		laneObj.ver_right_p1 =cvPoint(x, laneR.m.get()*x + laneR.c.get());
		laneObj.ver_right_p2 =cvPoint(x2, laneR.m.get() * x2 + laneR.c.get());

		line(lane_image,laneObj.ver_right_p1, laneObj.ver_right_p2, CV_RGB(255, 0, 255), 2, 8);
		//showIm("Left-Lane",lane_image);
		x = lane_image.cols * 0;
		x2 = lane_image.cols * 0.45f;
		laneObj.ver_left_p1 = cvPoint(x, laneL.m.get()*x + laneL.c.get());
		laneObj.ver_left_p2 = cvPoint(x2, laneL.m.get() * x2 + laneL.c.get());
		line(lane_image, laneObj.ver_left_p1, laneObj.ver_left_p2, CV_RGB(255, 0, 255), 2, 8);
		showIm("Left and Right-Lane",lane_image);

		printf("Vertical Right-Lane Points: (%d,%d) & (%d,%d)\n",laneObj.verRP1.x,laneObj.verRP1.y,
				laneObj.verRP2.x,laneObj.verRP2.y);
		printf("Vertical Left-Lane Points: (%d,%d) & (%d,%d)\n",laneObj.verLP1.x,laneObj.verLP1.y,
				laneObj.verLP2.x,laneObj.verLP2.y);
	}
	else{
		laneObj.hor_lower_p1 = cvPoint(x, H_lower.m.get()*x + H_lower.c.get());
		laneObj.hor_lower_p2 = cvPoint(x2, H_lower.m.get() * x2 + H_lower.c.get());
		line(lane_image,laneObj.hor_lower_p1, laneObj.hor_lower_p2,CV_RGB(255, 0, 255), 2, 8);
		//showIm("Left-Lane",lane_image);
		x = lane_image.cols * 0;
		x2 = lane_image.cols * 0.45f;
		laneObj.hor_upper_p1 = cvPoint(x, H_upper.m.get()*x + H_upper.c.get());
		laneObj.hor_upper_p2 = cvPoint(x2, H_upper.m.get() * x2 + H_upper.c.get());
		line(lane_image, laneObj.hor_upper_p1, laneObj.hor_upper_p2	, CV_RGB(255, 0, 255), 2, 8);
		showIm("Upper and Lower-Lane",lane_image);

		// Scale the values for original image
		laneObj.hor_lower_p1 = cvPoint(laneObj.hor_lower_p1.y,laneObj.hor_lower_p1.x);
		laneObj.hor_lower_p2 = cvPoint(laneObj.hor_lower_p2.y,laneObj.hor_lower_p2.x);
		laneObj.hor_upper_p1 = cvPoint(laneObj.hor_upper_p1.y,laneObj.hor_upper_p1.x);
		laneObj.hor_upper_p2 = cvPoint(laneObj.hor_upper_p2.y,laneObj.hor_upper_p2.x);
		printf("After modification:\n");
		printf("Vertical Upper-Lane Points: (%d,%d) & (%d,%d)\n",laneObj.horizUpP1.x,laneObj.horizUpP1.y,
				laneObj.horizUpP2.x,laneObj.horizUpP2.y);
		printf("Vertical Lower-Lane Points: (%d,%d) & (%d,%d)\n",laneObj.horizLowP1.x,laneObj.horizLowP1.y,
				laneObj.horizLowP2.x,laneObj.horizLowP2.y);
	}
}

void calulateControlSignal(){
	// Variations -> m_leftVar, c_leftVar, m_rightVar, c_rightVar;
	// Current result -> laneL.m.get(),laneL.c.get(), laneR.m.get(), laneR.c.get()
	// Basic control
#if VIDEO_INPUT // If input is video signal
#else
	float m_L = laneL.m.get();
	float c_L = laneL.c.get();
	float m_R = laneR.m.get();
	float c_R = laneR.c.get();

		//laneObj.horixLowValid,laneObj.horixUpValid,laneObj.verleftValid,laneObj.verRightValid;	
	if(!laneObj.horixLowValid && !laneObj.horixUpValid && 
			!laneObj.verleftValid && !laneObj.verRightValid){ // No lane
		printf("Control: NONE  (No Lane.! Follow pervious path.)\n");
	}
	else if(!laneObj.horixLowValid && !laneObj.horixUpValid){ // Only Vertical lane
		printf("Control: ?  (Verical Lane.!)\n");
	} 
	else if(!laneObj.verleftValid && !laneObj.verRightValid){ // Only horizontal lane
		printf("Control: ?  (Horizontal Lane.!)\n");
	}
	else{		// Both horizontal and verical components are present.
		printf("Control: ?  (All Lane.!)\n");
	}


#endif	
}

int main(int argc, char** argv )
{
#if VIDEO_INPUT 
	VideoCapture cap(0); // open the default camera
    if(!cap.isOpened()){  // check if we succeeded
     	printf("Unable to open camera.!\n");
        return -1;
    }
    // Read image
while(1){
    Mat image;
    waitKey(3000);
    cap >> image;
#else	
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    // Read image
{
    Mat image;
    image = imread( argv[1], 1 );
#endif
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
	showIm("Original Image",image);
	
	// Configure
	size_x = image.cols;
	size_y = image.rows;
	size_z = 3;
	ROI_w = size_x;
	ROI_h = size_y/3;
	ROI_x = 0;
	ROI_y = size_y-ROI_h;
	ROI =  cvRect(ROI_x,ROI_y,ROI_w,ROI_h);

	// ROI processing (crop+gray_scale)
	Mat im_roi, im_gray, im_display1,im_display2;
	im_roi = image(ROI).clone();
	im_display1 = image(ROI).clone();
	im_display2 = image(ROI).clone();

	cvtColor(im_roi, im_gray, CV_BGR2GRAY);
	showIm("Image-ROI",im_gray);
	
	// Pre-process (Gaussian blur and edge detect)
	Mat im_edge;
	//GaussianBlur(im_gray,im_gray,Size( 3, 3 ), 0, 0 );
	Canny(im_gray, im_edge, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);
	showIm("Image-Edges",im_edge);
    
	// Hough Transform
	std::vector<Vec4i> lines;
	double rho = 1;
	double theta = CV_PI/180;
	HoughLinesP(im_edge, lines, rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);
	
	// Lane detection
	// Vertical lane
	findLane(&lines,im_edge,im_roi,false);

	// Horizontal lane
	Mat im_edge_t = im_edge.t();
	Mat im_roi_t = im_roi.t();
	std::vector<Vec4i> lines_t;
	HoughLinesP(im_edge_t, lines_t, rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);
	findLane(&lines_t,im_edge_t,im_roi_t,true);
	
	// Projections
	line(im_display1, laneObj.hor_upper_p1, laneObj.hor_upper_p2, CV_RGB(255, 0, 255), 2, 8);
	line(im_display1, laneObj.hor_lower_p1, laneObj.hor_lower_p2, CV_RGB(255, 0, 255), 2, 8);
	line(im_display1, laneObj.ver_left_p1,  laneObj.ver_left_p2, CV_RGB(255, 0, 255), 2, 8);
	line(im_display1, laneObj.ver_right_p1, laneObj.ver_right_p2, CV_RGB(255, 0, 255), 2, 8);
	showIm("Lane projections",im_display1);

	// Final Lanes
	line(im_display2, laneObj.horizUpP1, laneObj.horizUpP2, CV_RGB(255, 0, 255), 2, 8);
	line(im_display2, laneObj.horizLowP1, laneObj.horizLowP2, CV_RGB(255, 0, 255), 2, 8);
	line(im_display2, laneObj.verLP1,  laneObj.verLP2, CV_RGB(255, 0, 255), 2, 8);
	line(im_display2, laneObj.verRP1, laneObj.verRP2, CV_RGB(255, 0, 255), 2, 8);
	showIm("Final Lanes",im_display2);

	// Control Logic
	calulateControlSignal();

	waitKey(0);
}
    return 0;
}
