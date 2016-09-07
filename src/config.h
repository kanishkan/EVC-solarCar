//-------------------------------------------------------------------------------------------------- 
// EINDHOVEN UNIVERSITY OF TECHNOLOGY
//-------------------------------------------------------------------------------------------------- 
// Author: Kanishkan
// Team 18
// Date and place: July 24th, 2016. Eindhoven, Netherlands
//-------------------------------------------------------------------------------------------------- 
//-------------------------------------------------------------------------------------------------- 
//  AVAILABLE FUNCTIONS/DEFINITIONS:
//  	Structure for all data types used in Lane detection and Control Logic
//-------------------------------------------------------------------------------------------------- 
#ifndef __IPM_H__
#define __IPM_H__

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

// Control Paramerter
#define LANE_ROTATION_ERROR_TH 15.0

//Sign Debug
#define SIGN_DBG 0
#define SIGN_DBGC 1	// Critical DBG
#define IMG_ON 0

//Lane Debug
#define EDGE_IMG 1
#define LINE_DBG 0
#define LINE_IMG 1
#define LINEF_DBG 0
#define LINEF_IMG 1
#define SETPT_DBG 1
#define SETPT_IMG 1

#define CONT_DBG 1
#define CONT_IMG 1

#define CONF_DBG 0
#define EN_PWR_LOG 1

// Lane Pattern Debug
#define LANE_PATTERN_DBG 0

enum{

	//Camera Input
	CAPTURE_WIDTH = 800,	//1296,
	CAPTURE_HEIGHT = 600,	//730,

	CANNY_MIN_TRESHOLD = 300,		// edge detector minimum hysteresis threshold
	CANNY_MAX_TRESHOLD = 400,		// edge detector maximum hysteresis threshold
	
	HOUGH_TRESHOLD = 30,			// Accumulator threshold parameter. 
	HOUGH_MIN_LINE_LENGTH = 150,	// Minimum line length. Line segments shorter than that are rejected.
	HOUGH_MAX_LINE_GAP = 10,		// The maximum gap between two points to be considered in the same line.

	// Line classification
	LINE_SEP_ANGLE = 11,				// In degrees
	HORIZ_LANE_MIN_DIST = 100,			// Minimum distance of Horiz lane for including in control 	(Cols-dist)
	HORIZ_LANE_CRITICAL_DIST = 100,		// Crtical position after which it requires sudden turn  	(Cols-dist)
	HORIZ_LANE_LEFT_REJECTION = 50,		// Left border
	HORIZ_LANE_RIGHT_REJECTION = 350,	// Right border

	VERT_LANE_SEPERATION = 100,

	// Sign Post Filter
	POST_ANGLE = 5,						// 	Perndicular lines are posts, so remove +-5 degree line if it does not satisfy line property

	// Sign window
	SIGN_TEMPORAL_WIN = 3,
	SIGN_TEMPORAL_TH  = 1,
	SIGN_AREA_TH = 5000,
	// Lane
	SS_LANE_ANGLE = 32,					// Steady state angle(Degree) {angle when parallel lines are viewed in camera}

	// PID Control
	STEP_DIST = 1,
};

enum LineType{
	HORIZ_LINE,
	VERTICAL_LINE,
};

enum TurnType{
	LINEAR,			// Gradual turn (smooth) *Not yet implemented
	CRITICAL,		// For suddent turn
};

/*
	Lane Structure:
		Start and End point of Lane : p1, p2
		Lane Property: Angle-angle, slope-m, Y intersect-c (for line equation: Y= mX+c)
		Lane Border Points: intersectLower- Intersect at lower border,  intersectUpper-Intersect at upper border 
							(For horizontal it is reused for left and right borders)
*/
struct Lane{
	Lane(){}
	Lane(CvPoint p1l, CvPoint p2l, float ang, float ml, float cl, \
		float intrsecL, float intrsecU, enum LineType L_typ, float lineDist, float v): \
		p1(p1l),p2(p2l),angle(ang),m(ml),c(cl), \
		intersectLower(intrsecL), intersectUpper(intrsecU), L_type(L_typ),lineDistance(lineDist),valid(v){}
	CvPoint p1;
	CvPoint p2;
	float angle, m, c;
	float intersectLower;
	float intersectUpper;
	float lineDistance;
	enum LineType L_type;	// Lane type (After analysing )
	int valid;				// If the lane is valid
};

enum DirectionVector{		// For sign, and direction
	STRAIGHT,
	LEFT,
	RIGHT,
	BACK,
	UTURN,
	STOP,
	NONE,
};

enum Confidence{ 			// Set-point confidence level
	HIGH,
	MEDIUM,
	LOW,
	POOR,
}; 					

enum LanePattern{			// For history
	NORMAL,
	HORIZONTAL,
	L_LANE,
	R_LANE,
	CORNER,
	LINE_BREAK,
	UNKNOWN,
	ERROR,
	NOLANE
};

enum PIDRotation{			// For safety layer
	TRANSLATE,
	ROTATE,
};

struct ControlInfo{			// Control Information
	enum DirectionVector direction;
	unsigned int angle;
	enum DirectionVector trafficSign;
	enum Confidence dirCounter;			// Lane confidence
	enum Confidence signCounter;		// Sign Confidence
	int valid;
	enum LanePattern lanePatern;
	enum DirectionVector nextDirection; 
	enum PIDRotation pidControlMsgType;
};

struct signInfo{
	enum DirectionVector direction;
	float area;
	int confidanceCount; 	// For temportal redudancy
};

// For printing
char result[100];
#endif /*__IPM_H__*/
