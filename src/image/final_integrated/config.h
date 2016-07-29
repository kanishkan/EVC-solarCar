#ifndef __IPM_H__
#define __IPM_H__

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

//Sign DBG
#define SIGN_DBG 0
#define SIGN_DBGC 1	// Critical DBG

//Lane DBG
#define EDGE_IMG 0
#define LINE_DBG 0
#define LINE_IMG 0
#define LINEF_DBG 0
#define LINEF_IMG 1
#define SETPT_DBG 1
#define SETPT_IMG 1

#define CONT_DBG 1
#define CONT_IMG 1

enum{
	CANNY_MIN_TRESHOLD = 300,		// edge detector minimum hysteresis threshold
	CANNY_MAX_TRESHOLD = 400,		// edge detector maximum hysteresis threshold
	
	HOUGH_TRESHOLD = 30,			// Accumulator threshold parameter. 
	HOUGH_MIN_LINE_LENGTH = 120,	// Minimum line length. Line segments shorter than that are rejected.
	HOUGH_MAX_LINE_GAP = 50,		// The maximum gap between two points to be considered in the same line.

	// Line classification
	LINE_SEP_ANGLE = 11,			// In degrees
	HORIZ_LANE_MIN_DIST = 100,		// Minimum distance of Horiz lane for including in control 	(Cols-dist)
	HORIZ_LANE_CRITICAL_DIST = 50,	// Crtical position after which it requires sudden turn  	(Cols-dist)
	HORIZ_LANE_LEFT_REJECTION = 50,	// Left border
	HORIZ_LANE_RIGHT_REJECTION = 350,// Right border

	VERT_LANE_SEPERATION = 100,

	// Sign window
	SIGN_TEMPORAL_WIN = 5,
	SIGN_TEMPORAL_TH  = 3,

	// Lane
	SS_LANE_ANGLE = 22,	// Steady state angle(Degree)

	// PID
	STEP_DIST = 5,
};

enum LineType{
	HORIZ_LINE,
	VERTICAL_LINE,
};

enum TurnType{
	LINEAR,			
	CRITICAL,		// For suddent turn
};

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
	enum LineType L_type;
	int valid;
};

enum DirectionVector{
	STRAIGHT,
	LEFT,
	RIGHT,
	BACK,
	UTURN,
	STOP,
	NONE,				// For sign
};

enum Confidence{
	HIGH,
	MEDIUM,
	LOW,
	POOR,
};
enum LanePattern{
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
enum PIDRotation{
	TRANSLATE,
	ROTATE,
};
struct ControlInfo{
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
//struct Tracking{
//	struct ControlInfo currentControl;
//	struct ControlInfo previousFrame;
//	int valid;
//}

// For printing

char result[100];
#endif /*__IPM_H__*/
