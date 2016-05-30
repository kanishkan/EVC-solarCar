#ifndef CONF_H
#define CONF_H
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

//Input image config
int size_x;
int size_y;
int size_z;

// ROI Configuration
int ROI_w;
int ROI_h;
int ROI_x;
int ROI_y;

enum{
	CANNY_MIN_TRESHOLD = 300,		// edge detector minimum hysteresis threshold
	CANNY_MAX_TRESHOLD = 400,		// edge detector maximum hysteresis threshold
	
	HOUGH_TRESHOLD = 30,			// Accumulator threshold parameter. 
	HOUGH_MIN_LINE_LENGTH = 100,	// Minimum line length. Line segments shorter than that are rejected.
	HOUGH_MAX_LINE_GAP = 50,		// The maximum gap between two points to be considered in the same line.

	SCAN_STEP = 5,					// in pixels
	LINE_REJECT_DEGREES = 10,		// in degrees
    BW_TRESHOLD = 250,				// edge response strength to recognize for 'WHITE'
    BORDERX = 10,					// px, skip this much from left & right borders
	MAX_RESPONSE_DIST = 5,			// px
	
	VOTE_LINE_WIDTH = 15, // Vk-verify
};

enum TURN{
	NONE = 0,
    LEFT=1,
    RIGHT=2,
    END=3,
};

struct Lane{
	Lane(){}
	Lane(CvPoint p1l, CvPoint p2l, float ang, float ml, float cl): p1(p1l),p2(p2l),angle(ang),m(ml),c(cl){}
	CvPoint p1;
	CvPoint p2;
	float angle, m, c;
};

struct LanePair{
	LanePair(){}
	LanePair(Lane l1, Lane l2,int id1,int id2,bool horiz): L1(l1),L2(l2),L1_id(id1),L2_id(id2),isHorizLane(horiz){}
	Lane L1;
	Lane L2;
	int L1_id;
	int L2_id;
	bool isHorizLane;
};

float dist(Lane l) 
{ 
	CvPoint line0 = l.p1;
	CvPoint line1 = l.p2;
	CvPoint dir; 
	dir.x = line0.x-line1.x;
	dir.y = line0.y-line1.y;
	return sqrtf(dir.x*dir.x + dir.y*dir.y); 
}

float xInterSect(Lane l,int h){
	return ((h-1)-l.c)/l.m;
}

float yInterSect(Lane l,int w){
	return (w/2)*l.m +l.c; 
}
float interSectPoint(Lane L1, Lane L2){
	return (L2.c-L1.c)/(L1.m-L2.m);
}
#endif