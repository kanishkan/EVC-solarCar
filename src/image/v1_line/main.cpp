#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

//Input image config
const int size_x = 800;
const int size_y = 600;
const int size_z = 3;

// ROI Configuration
const int ROI_w = 800;
const int ROI_h = 300;
const int ROI_x = size_x-800;
const int ROI_y = size_y-ROI_h;
Rect ROI =  cvRect(ROI_x,ROI_y,ROI_w,ROI_h);

// Parameters
enum{
	CANNY_MIN_TRESHOLD = 1,	  // edge detector minimum hysteresis threshold
	CANNY_MAX_TRESHOLD = 100, // edge detector maximum hysteresis threshold
	
	HOUGH_TRESHOLD = 50,		// line approval vote threshold
	HOUGH_MIN_LINE_LENGTH = 50,	// remove lines shorter than this treshold
	HOUGH_MAX_LINE_GAP = 100,   // join lines to one with smaller than this gaps
};

// Supporting function
void findLane(std::vector<Vec4i> *lines, Mat edge,Mat original);

void showIm(String title, Mat im){
	namedWindow(title, WINDOW_AUTOSIZE );
    imshow(title, im);
}

int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
	
	// Read image
    Mat image;
    image = imread( argv[1], 1 );

    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
	showIm("Original Image",image);
	
	// ROI processing (crop+gray_scale)
	Mat im_roi, im_gray;
	im_roi = image(ROI).clone();
	cvtColor(im_roi, im_gray, CV_BGR2GRAY);
	showIm("Image-ROI",im_gray);
	
	// Pre-process (Gaussian blur and edge detect)
	Mat im_edge;
	GaussianBlur(im_gray,im_gray,Size( 5, 5 ), 0, 0 );
	Canny(im_gray, im_edge, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);
	showIm("Image-Edges",im_edge);
    
	// Hough Transform
	std::vector<Vec4i> lines;
	double rho = 1;
	double theta = CV_PI/180;
	HoughLinesP(im_edge, lines, rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);
	
	// Lane detection
	//findLane(lines,im_edge,im_roi);
	waitKey(0);
    return 0;
}

void findLane(vector<Vec4i> *lines, Mat edge,Mat original){
	// classify lines to left/right side
	std::vector<Lane> left, right;
	
}