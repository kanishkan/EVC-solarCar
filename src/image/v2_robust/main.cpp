#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "config.h"

using namespace cv;
using namespace std;
Rect ROI;

//Input
#define VIDEO_INPUT 0

void showIm(String title, Mat im);
void findLane(vector<Vec4i> *lines, Mat edge, Mat original);
void filterLane(std::vector<Lane> lanes, Mat edges);
void findPosition(std::vector<LanePair> L_Pair, Mat edges);

int main(int argc, char** argv )
{
	Mat image;
#if VIDEO_INPUT 
	VideoCapture cap(0); // open the default camera
    if(!cap.isOpened()){  // check if we succeeded
     	printf("Unable to open camera.!\n");
        return -1;
    }
while(1){
    waitKey(3000);
    cap >> image;
#else	
    if ( argc == 2 ){
         image = imread( argv[1], 1 );
		 printf("Input: %s\n", argv[1]);
	}
	else{
		char imgeLocation[100] = "..\\raw_case\\turn_left.jpg";
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
	Mat im_roi, im_gray, im_hist, im_display1,im_display2;
	cvtColor(image, im_gray, CV_BGR2GRAY);
	equalizeHist( im_gray, im_hist );				// vk-hist is not used

	im_roi = im_gray(ROI).clone();
	im_display1 = image(ROI).clone();
	im_display2 = image(ROI).clone();

	
	showIm("Image-ROI",im_roi);
	
	// Pre-process (Gaussian blur and edge detect)
	Mat im_edge;
	GaussianBlur(im_roi,im_roi,Size( 3, 3 ), 0, 0 );
	Canny(im_roi, im_edge, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);
	showIm("Image-Edges",im_edge);

	// Hough Transform
	std::vector<Vec4i> lines;
	double rho = 1;
	double theta = CV_PI/180;
	HoughLinesP(im_edge, lines, rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);

	// Lane detection
	findLane(&lines,im_edge,im_roi);

}
	waitKey(0);
}

// Find lanes
void findLane(vector<Vec4i> *lines, Mat edge, Mat original ){
	std::vector<Lane> lane;
	CvPoint p1 = CvPoint();
	CvPoint p2 = CvPoint();
	Mat res_im = Mat::zeros( original.rows, original.cols, CV_8UC3 );

	for	(int i=0; i<lines->size(); i++) {
		p1.x = lines->at(i)[0];
		p1.y = lines->at(i)[1];
		p2.x  = lines->at(i)[2];
		p2.y = lines->at(i)[3];

		int dx = p2.x - p1.x;
		int dy = p2.y - p1.y;
		float angle = atan2f(dy, dx) * 180/CV_PI; 		// Arc tangent (hough angle)
		dx = (dx == 0) ? 1 : dx; // prevent DIV/0! 
		float m = dy/(float)dx;
		float c = p1.y - m*p1.x;

		m = (m==0)?1:m;

		lane.push_back(Lane(p1,p2,angle, m, c));
		line(res_im, p1, p2, CV_RGB(i*50, 50, 200), 2, 8);
		printf("Lane-%d: (%d,%d) (%d,%d) \n",i,p1.x,p1.y,p2.x,p2.y);
	}
	showIm("Image-Hough Lines",res_im);

	filterLane(lane, edge);
}

bool subSet(Lane *l1,Lane *l2,int h, int w){
	float m_dev = abs(l1->angle-l2->angle);
	float line1_x = ((h-1)-l1->c)/l1->m;
	float line2_x = ((h-1)-l2->c)/l2->m;
	float c_dev = fabs(line1_x-line2_x);			// Distance between parallel lines

	float ANGLE_VAR = 0.5;
	float C_VAR = 10;

	if(m_dev < ANGLE_VAR && c_dev<C_VAR){		// Independednt vector
		return false;
	}
	int LINE_DIST_x = 10;
	int LINE_DIST_y = 10;

	// If it is subset, return true to remove it from list
	if(l1->p1.y <= l2->p1.y && l1->p2.y <= l2->p2.y ){ // Subset in In y-direction
		return true;
	}
	// If it is super set, return false.
	else if(l1->p1.y >= l2->p1.y && l1->p2.y >= l2->p2.y){
	}
	// If it is disconnected vectors(same direction & position), update L2 vector length to sum of two vecotrs. And return true to remove L1
	else if(l1->p1.y <= l2->p1.y && l1->p2.y >= l2->p2.y){

	}
	else if(l1->p1.y >= l2->p1.y && l1->p2.y <= l2->p2.y){

	}
}

void filterLane(std::vector<Lane> lanes_org, Mat edges){
	int w = edges.cols;
	int h = edges.rows;
	float HOIZ_LINE_ANGLE_MIN = 45;

	// Duplicate data
	std::vector<Lane> lanes;
	for(int i=0;i<lanes_org.size();i++)
		lanes.push_back(lanes_org[i]);

	int* valid = new int[lanes.size()];
	for(int i=0;i<lanes.size(); i++)
		valid[i] = 0;

	// Lanes with same property (Pair of lines)
	int* votes = new int[lanes.size()];
	std::vector<LanePair> L_Pair;
	// Config
	float m_deviation = 1, c_deviation_min=15, c_deviation_max=40;
	for(int i=0;i<lanes.size(); i++)
		votes[i] = 0;

	// Note: Valid only If number of lines > 1  //vk-fix required in tracking logic
	for (int j=0; j<lanes.size(); j++) {
		for (int k=j+1; k<lanes.size(); k++) {
			float m_dev = abs(lanes[j].angle-lanes[k].angle);		// For parallel lines

			float line1_x;
			float line2_x;
			bool isHoriz = false;
			if(fabs(lanes[j].angle)>HOIZ_LINE_ANGLE_MIN || fabs(lanes[k].angle)>HOIZ_LINE_ANGLE_MIN){		// For vetical line -> x-intersection
				line1_x = xInterSect(lanes[j],h);		//((h-1)-lanes[j].c)/lanes[j].m;
				line2_x = xInterSect(lanes[k],h);		//((h-1)-lanes[k].c)/lanes[k].m;
			}
			else{							// For horizontal line -> y-intersection
				isHoriz = true;
				line1_x = yInterSect(lanes[j],w);		//(w/2)*lanes[j].m +lanes[j].c;			
				line2_x = yInterSect(lanes[k],w);		//(w/2)*lanes[k].m +lanes[k].c;
			}

			float c_dev   = fabs(line1_x-line2_x);					// Distance between parallel lines

			if(m_dev < m_deviation && c_dev>c_deviation_min && c_dev<c_deviation_max){
				votes[j]++;
				votes[k]++;
				
				if(votes[j]>1){										// j-lane is referenced again. Update k-lane to new value
					for(int pos = 0;pos<L_Pair.size();pos++){
						if(L_Pair[pos].L1_id == j ){				// Modify k-values
							if(dist(L_Pair[pos].L2) < dist(lanes[k])){	// Update to new lane  //vk-improve detection using vector operation
								L_Pair[pos].L2 = lanes[k];
							}
							else{									// Do nothing
							}
						}
					}
					votes[j]--;
				}
				
				else if(votes[k]>1){								// k-lane is referenced again. Update j-lane to new value
					for(int pos = 0;pos<L_Pair.size();pos++){
						if(L_Pair[pos].L2_id == k ){				// Modify j-values
							if(dist(L_Pair[pos].L1) < dist(lanes[j])){	// Update to new lane	//vk-improve detection using vector operation
								L_Pair[pos].L1 = lanes[j];
							}
							else{									// Do nothing
							}
						}
					}
					votes[k]--;
				}
				
				else												// First lane
					L_Pair.push_back(LanePair(lanes[j],lanes[k],j,k,isHoriz));

			}
		}
	}
	 
	// Results of lane classification
	Mat res_im = Mat::zeros( edges.rows, edges.cols, CV_8UC3 );
	
	for	(int i=0; i<L_Pair.size(); i++) {
		line(res_im, L_Pair.at(i).L1.p1, L_Pair.at(i).L1.p2, CV_RGB(i*50, 50, 200), 2, 8);
		line(res_im, L_Pair.at(i).L2.p1, L_Pair.at(i).L2.p2, CV_RGB(i*50, 50, 200), 2, 8);
		printf("Lanes after filter-%d: (%d,%d)(%d,%d) & (%d,%d)(%d,%d) Type: %s\n",i, L_Pair.at(i).L1.p1.x, L_Pair.at(i).L1.p1.y, L_Pair.at(i).L1.p2.x, L_Pair.at(i).L1.p2.y
																			, L_Pair.at(i).L2.p1.x, L_Pair.at(i).L2.p1.y, L_Pair.at(i).L2.p2.x, L_Pair.at(i).L2.p2.y,
																			L_Pair.at(i).isHorizLane?"Horizontal":"Vertical");
	}
	showIm("Image-Hough Lines after level-1 filter",res_im);

	findPosition(L_Pair,edges);
}

void findPosition(std::vector<LanePair> L_Pair, Mat edges){
	std::vector<Lane> leftLane, rightLane;
	std::vector<Lane> horizLane;
	Lane leftBorder, rightBorder;
	
	int MIN_LANE_DISTANCE = 50;
	int h = edges.rows;
	int w = edges.cols;
	int horizLaneCount = 0;
	int leftLaneCount = 0;
	int rightLaneCount = 0;

	// Classify vertical lines based on center of the image
	for(int i=0;i<L_Pair.size();i++){
		if(!L_Pair[i].isHorizLane){			// Vertical lane
			if( xInterSect(L_Pair[i].L1,h) <= w/2){
				leftLane.push_back(L_Pair[i].L2);
				leftLaneCount++;
			}
			else{
				rightLane.push_back(L_Pair[i].L1);
				rightLaneCount++;
			}
		}
		else{								// Horiz Lane
				horizLane.push_back(L_Pair[i].L1);
				horizLaneCount++;
		}
	}

	// Select lane which is close to center axis if we have both left and right lanes
	if(leftLaneCount>0 && rightLaneCount>0){			// Two lanes
		leftBorder = leftLane[leftLaneCount-1];
		rightBorder = rightLane[rightLaneCount-1];
		if(horizLaneCount == 0){	// No horizontal lane
			printf("Direction: Forward  (Logic: Two Verical Lanes)\n");
			if( fabs(leftBorder.m+rightBorder.m) <= 1 ){	// Both has same angle. 
				printf("Control: No-change\n");
			}
			else if(fabs(leftBorder.m) >= fabs(rightBorder.m)){	// Inclined towards left
				printf("Control: %f degree-Right\n",fabs(leftBorder.angle)-fabs(rightBorder.angle));
			}
			else{
				printf("Control: %f degree-Left\n",fabs(rightBorder.angle)-fabs(leftBorder.angle));
			}
		}
		else{
			// Find first horizontal line from bottom
			float horL = 0 ;
			for(int k=1;k<horizLaneCount;k++){
				if(yInterSect(horizLane[k],w) > yInterSect(horizLane[horL],w)){
					horL = k;
				}
			}
			CvPoint h_start,h_end;
			enum TURN horiz_ctrl = NONE;
			if(horizLane[horL].p1.x <w/2 && horizLane[horL].p2.x <w/2){	// Left lane
				h_start = (horizLane[horL].p1.x >= horizLane[horL].p2.x)?horizLane[horL].p1:horizLane[horL].p2;
				h_end = (horizLane[horL].p1.x >= horizLane[horL].p2.x)?horizLane[horL].p2:horizLane[horL].p1;
				horiz_ctrl = LEFT;
			}
			// Vk- add offset for second value to make it accurate
			else if((horizLane[horL].p1.x <w/2 && horizLane[horL].p2.x > w/2) || (horizLane[horL].p2.x <w/2 && horizLane[horL].p1.x > w/2)) {	
				// Reached end. 
				h_start = horizLane[horL].p1;
				h_end = horizLane[horL].p2;
				horiz_ctrl = END;
			}
			else{	// Right lane
				h_start = (horizLane[horL].p1.x >= horizLane[horL].p2.x)?horizLane[horL].p2:horizLane[horL].p1;
				h_end = (horizLane[horL].p1.x >= horizLane[horL].p2.x)?horizLane[horL].p1:horizLane[horL].p2;
				horiz_ctrl = RIGHT;
			}

			// Find shortest horizontal lane(y-direction)
			CvPoint leftPos = (leftBorder.p1.y<=leftBorder.p2.y)?leftBorder.p1:leftBorder.p2;
			CvPoint rightPos = (rightBorder.p1.y<=rightBorder.p2.y)?rightBorder.p1:rightBorder.p2;
			int minDist = (leftPos.y<=rightPos.y)?leftPos.y:rightPos.y;
			int maxDist = (leftPos.y>rightPos.y)?leftPos.y:rightPos.y;
			// Decide based on intersection of horizontal and vertical lane
			if(horiz_ctrl == END){
				if(fabs((float)leftPos.y-rightPos.y) > MIN_LANE_DISTANCE){
					if(leftPos.y<rightPos.y)
						printf("Direction: Straight for %d then Left (Logic: Vertical + horiz)\n",minDist);
					else
						printf("Direction: Straight for %d then Right (Logic: Vertical + horiz)\n",minDist);
				}
				else{
					if(maxDist-h_start.y >MIN_LANE_DISTANCE)
						printf("Direction: Straight for %d then LEFT/RIGHT (Logic: Horiz line+ Open verical)\n",minDist);
					else
						printf("Direction: Straight for %d then END (Logic: Horiz line+Closed Vertical)\n",minDist);
				}
			}
			else if(horiz_ctrl == RIGHT){
				printf("Direction: Straight then Right\n");
			}
			else if(horiz_ctrl == LEFT){
				printf("Direction: Straight then Left\n");
			}
		}
	}
	if(leftLaneCount == 0 && rightLaneCount >0){		// Only right lanes
	
	}

}
// Display image
void showIm(String title, Mat im){
	namedWindow(title, WINDOW_AUTOSIZE );
    imshow(title, im);
}