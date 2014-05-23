#include <iostream>
#include <stdio.h>
#include <iostream>
#include <ostream>
#include <stdlib.h>
#include <math.h>
#include <vector>

//Include Libraries
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

void detectLines(void);
void findcentre(int);
int findcentre2(int);
void drawLine(int);
int countLines(int);

Mat image, image_grey, image_filter, image_thresholded, image_canny;
int const threshold_value = 160;
int blur_value = 7;
int const max_BINARY_value = 255;
int direction[3]; // x,y,z using right handed co-ordinate system, + z rotate counter clockwise
//TODO: publish message
//TODO: currently there is a lot of overlap between functions, can speed up code by getting rid of the overlap & also using pointers
int main(int argc, char** argv) {
	VideoCapture cap("sample-course.avi");
	namedWindow("Display Image", CV_WINDOW_NORMAL);

	while(1){

	//VideoCapture cap VideoCapture(0); // open the default camera

	if(!cap.isOpened())  // check if we succeeded
    return -1;

	cap >> image;

	//Read image
	//image = imread(argv[1], 1);

	//Stop if no image data
	if (argc != 2 || !image.data) {
		printf("No image data \n");
		return -1;
	}
	//Create image_gray, gray_scaled version of image (needed for canny filter)
	cvtColor(image, image_grey, CV_RGB2GRAY);

	//Blur and threshold the image
	if (blur_value % 2 == 0) {
		blur_value = blur_value + 1;
	}
	medianBlur(image_grey, image_filter, blur_value);
	threshold(image_filter, image_thresholded, threshold_value,
			max_BINARY_value, THRESH_BINARY);

	//Canny Edge detection
	Canny(image_thresholded, image_canny, 50, 200, 3);

	// Detect lines
	detectLines();

	//Display image

	imshow("Display Image", image);

	//Display gray image
	namedWindow("Grey Image", CV_WINDOW_NORMAL);
	imshow("Grey Image", image_grey);

	//Display line image
	namedWindow("Lines", CV_WINDOW_NORMAL);
	imshow("Lines", image_thresholded);

	//Display Canny image
	namedWindow("Canny", CV_WINDOW_NORMAL);
	imshow("Canny", image_canny);

	// Wait for one ms, break if escape is pressed
	if(waitKey(1) == 27) break;
	}
	destroyAllWindows();
	return 0;
}


void detectLines(void) {
	int row = image.rows / 2-52;// starting row for checking direction
	int betweenRow = 10;
	int x = 0;
	int y = 0;
	int dx = 0;
	int dy = 0;
	//TODO: for... different horizontal lines to minimize noise, starting with just the centreline

	int numLines = 0;
	for (int i = 0; i <= 1; i++){
	//Draw lines on thresholded image, where it is being checked
	drawLine(row);
	//findcentre(row);
	numLines = countLines(row); // how many lanes are detected
	if (numLines == 2)
		{
		x = findcentre2(row); // if we detect two lines find the middle of the lane
		y = row;
	if(i == 0)
	{
		dx = x;
		dy = y;
	}
	if (i ==1)
	{
		dx = x-dx;
		dy =  dy-y;
		cout<<"Rise/Run: "<<dy<<"/"<<dx<<endl;
	}
	}
	row = row - betweenRow;
	}
}

//Draws a horizontal line at row
void drawLine(int row)
{
	Point pt1, pt2;
	pt1.x = 0;
	pt1.y = row;
	pt2.x = image.cols;
	pt2.y = row;
	line(image_canny, pt1, pt2, CV_RGB(250, 100, 255), 1, CV_AA);
}

//Detects where line are and highlights them using circles
void findcentre(int row) {
	//TODO: for pixels in line find high low changeovers calculate centre of white lines then centre of path
	int Mit;
	int sum = 0;
	int count = 0;
	int average;
	for (int i=0; i<image_thresholded.cols; i++){
		Mit = (image_thresholded.at<uchar>(row, i)) % 2; // had problems with data type of binary image. Modulo works to get either 1 or 0
		cout << Mit << endl;
		if (Mit > 0) {
			//Highlight area where circle is detected
			Point centre;
			centre.x = i;
			centre.y = row;
			circle(image_canny, centre, 5, CV_RGB(250, 100, 255), 1, 8, 0);
			sum = sum + i;
			count++;
		}
	}
	average = sum / count;
	//Put large dot in average of white lines
	Point centre;
	centre.x = average;
	centre.y = row;
	circle(image_canny, centre, 10, CV_RGB(250, 100, 255), 1, 8, 0);
}

//Improved version of find centre when exactly 2 lanes have been detected
int findcentre2(int row) {
	int Mit;
	int countWhite = 0;
	int centreWhite1=0;
	int centreWhite2=0;
	int centreLane;
	int transition = 0;
	int lastValue = (image_thresholded.at<uchar>(row, 0)) % 2;
	for (int i=0; i<image_thresholded.cols; i++)
	{
		Mit = (image_thresholded.at<uchar>(row, i)) % 2; // had problems with data type of binary image. Modulo works to get either 1 or 0
		if (lastValue != Mit)
		{

			if (transition == 0 ) countWhite = i;
			else if (transition == 1 ) {
				centreWhite1 = (countWhite + i)/2;
				cout<<"centreWhite1: "<<centreWhite1<<endl;
				countWhite = 0;
			}
			else if (transition == 2 ) countWhite = i;
			else if (transition == 3 ) {
				centreWhite2 = (countWhite + i)/2;
				cout<<"centreWhite2: "<<centreWhite2<<endl;
				countWhite = 0;
			}
			transition++;
		}
		lastValue = Mit;
	}
    centreLane = (centreWhite1 + centreWhite2)/2;
    cout<<"CentreLane:"<<centreLane<<endl;
	//Put large dot in average of white lines
	Point centre;
	centre.x = centreLane;
	centre.y = row;
	circle(image_canny, centre, 20, CV_RGB(250, 100, 255), 1, 8, 0);
	return centreLane;
}
// Counts the number of lines seen across the image
int countLines(int row){
	int Mit;
	int lineCount = 0;
	int transitionCount = 0;
	int lastValue = (image_thresholded.at<uchar>(row, 0)) % 2;
	for (int i=0; i<image_thresholded.cols; i++){
			Mit = (image_thresholded.at<uchar>(row, i)) % 2;
			if (Mit != lastValue) {
				transitionCount++;
			}
			lastValue = Mit;
		}
	lineCount = ceil(transitionCount/2.0);
	cout<<"Number of Transitions: "<<transitionCount<<endl;
	cout<<"Number of Lines: "<<lineCount<<endl;
	return lineCount;
}


