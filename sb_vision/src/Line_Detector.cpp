#include <iostream>
#include <stdio.h>
#include <iostream>
#include <ostream>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <fstream>
#include <limits>

//Include Libraries
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>

using namespace cv;
using namespace std;

void detectLines(void);
void findcentre(int);
int findcentre1(int);
int findcentre2(int);
void drawLine(int);
int countLines(int, bool);
void dump(const Mat&, const char*);
Mat showHistogram(const Mat&);
Mat showHistogram4(const Mat &inImage, int, int);
void showHSVHistograms(Mat);
void displayWindows(void);
void filterImage(void);
void getDirection(void);
float calculateDirection1();
void convert2Twist(void);

Mat image, image_grey, image_filter, image_thresholded, image_canny, image_blur,
		image_blur2, image_direction;
Mat image_H, image_S, image_V, image_histo, image_HThresh, image_SThresh,
		image_VThresh;
Mat histogram_H, histogram_S, histogram_V;
//float direction[3] = { 0, 0, 0 }; // x,y,z using right handed co-ordinate system, + z rotate counter clockwise
int const threshold_value = 180;
int blur_value = 2;
int const max_BINARY_value = 255;
int dx = 0;
int dy = 0;

//TODO: publish message

int main(int argc, char** argv) {
	cout << "Running" << endl;

	//Initialize camera
	//VideoCapture cap(0); // open the default camera
	VideoCapture cap("sample-course.avi");
	//VideoCapture cap("roadsample.mov");
	if (!cap.isOpened()) {
		cout << " Camera not opened";
		return -1;
	}

	while (1) {

		 //create a window called "MyVideo"
		//Read image
		cap >> image; //from video

		if (!cap.read(image)) //if not success, break loop
				{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		cout << "Image read" << endl;

		filterImage();

		//detectLines();

		getDirection();

		displayWindows();

		// Wait for user input
		waitKey(0); //use waitKey(0); to enter "debugging" mode
		if (waitKey(100) == 27)
			break; // Wait for one ms, break if escape is pressed
	}
	destroyAllWindows();

	return 0;
}

void filterImage(void) {

	//Blur Image
	if (blur_value % 2 == 0)
		blur_value = blur_value + 1;
	medianBlur(image, image_blur, blur_value);

	showHSVHistograms(image_blur);

	//Create grey-scaled version of image
	cvtColor(image_blur, image_grey, CV_RGB2GRAY);

	//Blur the image
	if (blur_value % 2 == 0)
		blur_value = blur_value + 1;
	medianBlur(image_grey, image_blur2, blur_value);

	//Threshold the image: 3 different options
	//Regular threshold
	threshold(image_blur2, image_thresholded, threshold_value, max_BINARY_value,
			THRESH_BINARY);
	//Adaptive threshold
	//adaptiveThreshold(image_blur2, image_thresholded,255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY,71, 15);
	//Otsu threshold
	//threshold(image_blur2, image_thresholded, threshold_value, 255 , THRESH_OTSU|THRESH_BINARY  );
	image_direction = image_thresholded.clone();
	//Canny Edge detection
	Canny(image_thresholded, image_canny, 50, 200, 3);

	//Testing out hough lines again, however having problems with formatting
	/*vector<Vec2f> lines;
		Mat Hough;
		Mat convert;
		cv::cvtColor(image_thresholded,convert, COLOR_GRAY2RGB);
		HoughLines(convert, Hough,1,CV_PI/180,5,0,0);
		for (size_t i = 0; i < lines.size(); i++) {
			float rho = lines[i][0], theta = lines[i][1];
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a * rho, y0 = b * rho;
			pt1.x = cvRound(x0 + 1000 * (-b));
			pt1.y = cvRound(y0 + 1000 * (a));
			pt2.x = cvRound(x0 - 1000 * (-b));
			pt2.y = cvRound(y0 - 1000 * (a));
			line(Hough, pt1, pt2, Scalar(0, 250, 255), 3, CV_AA);
		}

	imshow("Hough",Hough);*/
}

void getDirection(void) {

	int rows2Check = 2;
	int distanceBetweenRows = image.rows / 10;
	int const startRow = image.rows / 2 + distanceBetweenRows; //TODO: adjust for camera angle
	int row = startRow;

	Point points[rows2Check][4];
	int transitions[2];

	int lastValue = (image_thresholded.at<uchar>(row, 0)) % 2;
	int currentValue = 0;

	int state = 1; // 0 we are out to the left, 1 we are inside the lines, 2 we are to the right of the lines

	image_direction = image_thresholded.clone();

	//Check how many transitions occur in each row and put into array
	for (int i = 0; i < rows2Check; i++) {
		transitions[i] = countLines(row, 1);
		row = row - distanceBetweenRows;
	}

	row = startRow;

	/* Categorize each transition
	 1. left side of left line
	 2. right side of left line
	 3. left side of right line
	 4. right side of right line
	 */

	for (int i = 0; i < rows2Check; i++) {

		//Initialize variables
		lastValue = (image_thresholded.at<uchar>(row, 0)) % 2;
		int left = 0;
		Point centre;


		if (transitions[i] == 0) {
			cout << "Error: No lines detected" << endl;
		} else if (transitions[i] > 4) {
			cout << "Error: More than 2 lines detected" << endl; // consider adjusting thresholding values here
		} else {
			for (int f = 0; f < image_thresholded.cols; f++) {
				currentValue = image_thresholded.at<uchar>(row, f) % 2;
				if (currentValue != lastValue) {
					cout << "Transition at x = " << f << ", y = " << row
							<< endl;
					centre.x = f;
					centre.y = row;
					cout<<"Point at = "<< centre << endl;
					circle(image_direction, centre, 20, CV_RGB(250, 100, 255), 1,
							8, 0);
					cout << "Current Value: " << currentValue << " Last Value"
							<< lastValue << endl;


				//Here comes the fun!!!! Value of black is 1 and value of white is 0
				// 1 transition
				if (transitions[i] == 1) {
					if ((lastValue == 1) && (currentValue == 0))
						points[i][1] = centre;
					else if ((lastValue == 0) && (currentValue == 1))
						points[i][2] = centre;
					left = 0;
				} else if (transitions[i] == 2) // this part right now just checks which side things are on
						{
					if (left == 0) {
						if ((lastValue == 1) && (currentValue == 0))
							points[i][1] = centre; //RoL
						else if ((lastValue == 0) && (currentValue == 1))
							if(f<=image_thresholded.cols/2) points[i][0] = centre; //LoL
					     	if (f>image_thresholded.cols/2) points[i][2] = centre; //LoR
						left = 1;
					} else if (left == 1) {
						if ((lastValue == 1) && (currentValue == 0))
							{
							if(f<=image_thresholded.cols/2) points[i][1] = centre; // RoL
							if(f>image_thresholded.cols/2)points[i][3] = centre; //LoL
							}
						else if ((lastValue == 0) && (currentValue == 1))
							points[i][2] = centre; //LoR
					}
				} else if (transitions[i] == 3) {
					if (left == 0) {
						if ((lastValue == 1) && (currentValue == 0))
							points[i][1] = centre; // RoL
						else if ((lastValue == 0) && (currentValue == 1))
							points[i][0] = centre; //LoL
						left = 1;
					} else if (left == 1) {
						if ((lastValue == 1) && (currentValue == 0))
							points[i][1] = centre; // RoL
						else if ((lastValue == 0) && (currentValue == 1))
							points[i][2] = centre; //LoR
						left = 2;
					} else if (left == 2) {
						if ((lastValue == 1) && (currentValue == 0))
							points[i][3] = centre; // RoL
						else if ((lastValue == 0) && (currentValue == 1))
							points[i][2] = centre; //LoR
					}
				} else if (transitions[i] == 4) {
					if (left == 0) {
						if ((lastValue == 0) && (currentValue == 1))
							points[i][0] = centre; //LoL
						left = 1;
					} else if (left == 1) {
						if ((lastValue == 1) && (currentValue == 0))
							points[i][1] = centre; // RoL
						left = 2;
					} else if (left == 2) {
						if ((lastValue == 0) && (currentValue == 1))
							points[i][2] = centre; // RoL
						left = 3;
					} else if (left == 3) {
						if ((lastValue == 1) && (currentValue == 0))
							points[i][3] = centre; // RoL
					}
				}
				}
				lastValue = currentValue;
			}
			drawLine(row);
			//cout<<"Left = " << left << "Row: " << row << endl;
			row = row - distanceBetweenRows;
			left = 0;
		}
	}

	//SUPER GHETTO STUFFS
    // 0 aka Left of Left -----------------------------
	float Xdata0[] = {points[0][0].x,1,points[1][0].x,1};
		Mat X0 = Mat(2, 2, CV_32F, Xdata0).clone();

		float Ydata0[] = {points[0][0].y,points[1][0].y};
		Mat Y0 = Mat(2, 1, CV_32F, Ydata0).clone();

		Mat B0 = Y0.clone();

		//want to stream line this by putting points directly into Mat, but for now just going to get something that works

		solve(X0,Y0,B0, DECOMP_LU);
		cout << "X0 = " << endl << X0 << endl;
		cout << "Y0 = " << endl << Y0 << endl;
		cout << "B0 = " << endl << B0 << endl;

		// Next we need to calculate how much we want to shift by: using the line equation obtained in matrix B we can calculate the
		// y-intercept
		float slope0 = B0.at<float>(0, 0);
		float yIntercept0 = B0.at<float>(1, 0);
		cout<<"slope0 = "<< slope0<<endl;
		cout <<"yIntercept0 = " << yIntercept0 <<endl;

		float xIntercept0 = - yIntercept0 / slope0;
		cout <<"xIntercept0 = " << xIntercept0 <<endl;

		if(yIntercept0 != 0){
			//Draw Line
			Point Intercept0 = Point(xIntercept0,0);
			Point Intercept640 = Point((640-yIntercept0)/slope0,640);
			line(image_direction, Intercept0, Intercept640, CV_RGB(250, 100, 255), 1, CV_AA);

		}
	// 1 aka Right of Left
		float Xdata1[] = {points[0][1].x,1,points[1][1].x,1};
			Mat X1 = Mat(2, 2, CV_32F, Xdata1).clone();

			float Ydata1[] = {points[0][1].y,points[1][1].y};
			Mat Y1 = Mat(2, 1, CV_32F, Ydata1).clone();

			Mat B1 = Y1.clone();

			//want to stream line this by putting points directly into Mat, but for now just going to get something that works

			solve(X1,Y1,B1, DECOMP_LU);
			cout << "X1 = " << endl << X1 << endl;
			cout << "Y1 = " << endl << Y1 << endl;
			cout << "B1 = " << endl << B1 << endl;

			// Next we need to calculate how much we want to shift by: using the line equation obtained in matrix B we can calculate the
			// y-intercept
			float slope1 = B1.at<float>(0, 0);
			float yIntercept1 = B1.at<float>(1, 0);
			cout<<"slope1 = "<< slope1<<endl;
			cout <<"yIntercept1 = " << yIntercept1 <<endl;

			float xIntercept1 = - yIntercept1 / slope1;
			cout <<"xIntercept1 = " << xIntercept1 <<endl;

			if (yIntercept1 != 0){
				//Draw Line
				Point Intercept0 = Point(xIntercept1,0);
				Point Intercept640 = Point((640-yIntercept1)/slope1,640);
				line(image_direction, Intercept0, Intercept640, CV_RGB(250, 100, 255), 1, CV_AA);
			}

	// 2 aka Left of Right
			float Xdata2[] = {points[0][2].x,1,points[1][2].x,1};
				Mat X2 = Mat(2, 2, CV_32F, Xdata2).clone();

				float Ydata2[] = {points[0][2].y,points[1][2].y};
				Mat Y2 = Mat(2, 1, CV_32F, Ydata2).clone();

				Mat B2 = Y2.clone();

				//want to stream line this by putting points directly into Mat, but for now just going to get something that works

				solve(X2,Y2,B2, DECOMP_LU);
				cout << "X2 = " << endl << X2 << endl;
				cout << "Y2 = " << endl << Y2 << endl;
				cout << "B2 = " << endl << B2 << endl;

				// Next we need to calculate how much we want to shift by: using the line equation obtained in matrix B we can calculate the
				// y-intercept
				float slope2 = B2.at<float>(0, 0);
				float yIntercept2 = B2.at<float>(1, 0);
				cout<<"slope2 = "<< slope2 <<endl;
				cout <<"yIntercept2 = " << yIntercept2 <<endl;

				float xIntercept2 = - yIntercept2 / slope2;
				cout <<"xIntercept2 = " << xIntercept2 <<endl;

				if (yIntercept2 != 0){

					//Draw Line
					Point Intercept0 = Point(xIntercept2,0);
					Point Intercept640 = Point((640-yIntercept2)/slope2, 640);
					line(image_direction, Intercept0, Intercept640, CV_RGB(250, 100, 255), 1, CV_AA);
				}

				float Xdata3[] = {points[0][3].x,1,points[1][3].x,1};
					Mat X3 = Mat(2, 2, CV_32F, Xdata3).clone();

					float Ydata3[] = {points[0][3].y,points[1][3].y};
					Mat Y3 = Mat(2, 1, CV_32F, Ydata3).clone();

					Mat B3 = Y3.clone();

					//want to stream line this by putting points directly into Mat, but for now just going to get something that works

					solve(X3,Y3,B3, DECOMP_LU);
					cout << "X3 = " << endl << X3 << endl;
					cout << "Y3 = " << endl << Y3 << endl;
					cout << "B3 = " << endl << B3 << endl;

					// Next we need to calculate how much we want to shift by: using the line equation obtained in matrix B we can calculate the
					// y-intercept
					float slope3 = B3.at<float>(0, 0);
					float yIntercept3 = B3.at<float>(1, 0);
					cout<<"slope3 = "<< slope3<<endl;
					cout <<"yIntercept3 = " << yIntercept3 <<endl;

					float xIntercept3 = - yIntercept3 / slope3;
					cout <<"xIntercept3 = " << xIntercept3 <<endl;

					if (yIntercept3 != 0){
						//Draw Line
						Point Intercept0 = Point(xIntercept3,0);
						Point Intercept640 = Point((640-yIntercept3)/slope3,640);
						line(image_direction, Intercept0, Intercept640, CV_RGB(250, 100, 255), 1, CV_AA);

					}

					// Crikey!
					float average = 0;
					float confidence = 0.1;
					if((yIntercept0 != 0) && (yIntercept1!= 0) && (yIntercept2!= 0) && (yIntercept3!= 0))
					{
						 average = (xIntercept0 + xIntercept1 + xIntercept2 +xIntercept3 )/4;
						 confidence = .99;
					}
					else if(((yIntercept0 != 0) && (yIntercept2!= 0)))
					{
						 average = (xIntercept0 + xIntercept2)/2;
						 confidence = .9;
					}
					else if(((yIntercept1 != 0) && (yIntercept3!= 0)))
					{
						 average = (xIntercept1 + xIntercept3)/2;
						 confidence = .9;
					}
					cout << "Average = " << average << endl;

					if((average < image_thresholded.cols/2 + 10) && (average > image_thresholded.cols/2 - 10))
					{
						cout << "GO STRAIGHT"<< endl;
					}
					else if((average < image_thresholded.cols/2 + 10))
					{
					cout << "GO LEFT"<< endl;
					}
					else if((average > image_thresholded.cols/2 + 10))
					{
					 cout << "GO RIGHT"<< endl;
					}

					if ((transitions[0] == 2) && (transitions[1] == 2))
					{
						average = (xIntercept0 + xIntercept1 + xIntercept2 + xIntercept3)/2;
						float slopeaverage = (slope0 + slope1 + slope2 + slope3)/2;
						if (average > image_thresholded.cols*2/3 + 10)
						{
							confidence = .5;
							cout << "GO LEFT"<< endl;
						}
						if (average > image_thresholded.cols*2/3 + 10)
						{
							confidence = 0.5;
							cout << "GO RIGHT"<< endl;
						}
					}
					if (average == 0) cout <<" No matching lines =("<< endl;
					cout << "Confidence = " << confidence << endl;
					cout << "Cols"<<image_thresholded.cols<<endl;
					cout<<"Rows"<< image_thresholded.rows<<endl;


}

/*
 float(calculateDirection){

 }
 */
float calculateDirection1(void) {
	int iterations = 2;
	int distanceBetweenRows = image.rows / 10;
	int startRow = image.rows / 2; //TODO: adjust for camera angle
	int row = startRow - distanceBetweenRows;

	Point point0;
	Point point1;
	int currentValue0;
	int lastValue0 = (image_thresholded.at<uchar>(row, 0)) % 2;
	int currentValue1;
	int lastValue1 = (image_thresholded.at<uchar>(row, 0)) % 2;

	for (int i = 0; i < image_thresholded.cols; i++) {
		//Mit = (image_thresholded.at<uchar>(row, i)) % 2;
		//if (Mit != lastValue) {
		//	transitionCount++;

		//lastValue = Mit;
	}
	return 0;
}

// Detect slopes
/*
 transitions = transitions/iterations; //may have rounding error here

 int lines = (transitions+1)/2*2;
 cout<< "Lines: " << lines << endl;

 if (transitions == 0)cout<< "Error: 0 lines detected, keep going in same direction"<<endl;
 else if (transitions == 1);
 else if (transitions == 2);
 else if (transitions == 3);
 else if (transitions == 4);
 else if (transitions >= 5)cout<< "Error: Noise, more than 2 lines detected"<<endl;

 cout<<"Direction: "<<direction<<endl;*/

void detectLines(void) {
	int row = image.rows / 2 - 52; // starting row for checking direction
	int betweenRow = 10;
	int x = 0;
	int y = 0;

	//TODO: for... different horizontal lines to minimize noise, starting with just the centreline

	int numLines = 0;
	for (int i = 0; i <= 1; i++) {
		//Draw lines on thresholded image, where it is being checked
		drawLine(row);
		//findcentre(row);
		numLines = countLines(row, 0); // how many lanes are detected
		if (numLines == 0)
			cout << "Error: No lines visible" << endl; //direction will stay as before
		if (numLines >= 3)
			cout << "Error: Noise, more than 3 lines detected" << endl; //direction will stay as before
		if (numLines == 2) {
			x = findcentre2(row); // if we detect two lines find the middle of the lane
			y = row;
		}
		if (numLines == 1) {
			x = findcentre1(row); // if we detect one lines find the middle of the lane
			y = row;
		}
		if (i == 0) {
			dx = x;
			dy = y;
		}
		if (i == 1) {
			dx = x - dx;
			dy = dy - y;
			cout << "Rise/Run: " << dy << "/" << dx << endl;
		}

		row = row - betweenRow;
	}
}
void displayWindows(void) {

	//Display image
	namedWindow("Display Image", CV_WINDOW_NORMAL);
	cvMoveWindow("Display Image", 400, 0);
	imshow("Display Image", image);

	//Display blur
	namedWindow("Blur", CV_WINDOW_NORMAL);
	cvMoveWindow("Blur", 720, 0);
	imshow("Blur", image_blur);

	//Display after blur
	namedWindow("After Blur", CV_WINDOW_NORMAL);
	cvMoveWindow("After Blur", 400, 300);
	imshow("After Blur", image_blur2);

	//Display binary threshold thresholded image
	namedWindow("Binary Threshold", CV_WINDOW_NORMAL);
	cvMoveWindow("Binary Threshold", 720, 300);
	imshow("Binary Threshold", image_thresholded);

	//Display Hue thresholded image
	namedWindow("Hue Otsu Threshold", CV_WINDOW_NORMAL);
	cvMoveWindow("Hue Otsu Threshold", 0, 300);
	imshow("Hue Otsu Threshold", image_HThresh);

	//Display Value thresholded image
	namedWindow("Value Threshold", CV_WINDOW_NORMAL);
	cvMoveWindow("Value Threshold", 720, 600);
	imshow("Value Threshold", image_VThresh);

	//Display H histogram
	namedWindow("H Histogram", CV_WINDOW_NORMAL);
	cvMoveWindow("H Histogram", 1150, 0);
	imshow("H Histogram", histogram_H);

	//Display S histogram
	namedWindow("S Histogram", CV_WINDOW_NORMAL);
	cvMoveWindow("S Histogram", 1150, 300);
	imshow("S Histogram", histogram_S);

	//Display V histogram
	namedWindow("V Histogram", CV_WINDOW_NORMAL);
	cvMoveWindow("V Histogram", 1150, 600);
	imshow("V Histogram", histogram_V);

	//Display direction image
	namedWindow("Canny", CV_WINDOW_NORMAL);
	cvMoveWindow("Canny", 0, 0);
	imshow("Canny", image_canny);

	//Display direction image
	namedWindow("Direction", CV_WINDOW_NORMAL);
	cvMoveWindow("Direction", 0, 0);
	imshow("Direction", image_direction);

}

//Draws a horizontal line at row
void drawLine(int row) {
	Point pt1, pt2;
	pt1.x = 0;
	pt1.y = row;
	pt2.x = image.cols;
	pt2.y = row;
	line(image_direction, pt1, pt2, CV_RGB(250, 100, 255), 1, CV_AA);
}

//Detects where line are and highlights them using circles
void findcentre(int row) {
	//TODO: for pixels in line find high low changeovers calculate centre of white lines then centre of path
	int Mit;
	int sum = 0;
	int count = 0;
	int average;
	for (int i = 0; i < image_thresholded.cols; i++) {
		Mit = (image_thresholded.at<uchar>(row, i)) % 2; // had problems with data type of binary image. Modulo works to get either 1 or 0
		cout << Mit << endl;
		if (Mit > 0) {
			//Highlight area where circle is detected
			Point centre;
			centre.x = i;
			centre.y = row;
			circle(image_direction, centre, 5, CV_RGB(250, 100, 255), 1, 8, 0);
			sum = sum + i;
			count++;
		}
	}
	average = sum / count;
	//Put large dot in average of white lines
	Point centre;
	centre.x = average;
	centre.y = row;
	circle(image_direction, centre, 10, CV_RGB(250, 100, 255), 1, 8, 0);
}

//Improved version of find direction when exactly 1 lane has been detected
int findcentre1(int row) {
	int Mit;
	int countWhite = 0;
	int centreWhite1 = 0;
	int centreLane;
	int transition = 0;
	int lastValue = (image_thresholded.at<uchar>(row, 0)) % 2;
	for (int i = 0; i < image_thresholded.cols; i++) {
		Mit = (image_thresholded.at<uchar>(row, i)) % 2; // had problems with data type of binary image. Modulo works to get either 1 or 0
		if (lastValue != Mit) {
			if (transition == 0)
				countWhite = i;
			else if (transition == 1) {
				centreWhite1 = (countWhite + i) / 2;
				cout << "centreWhite1: " << centreWhite1 << endl;
				countWhite = 0;
			}
			transition++;
		}
		lastValue = Mit;
	}
	if (centreWhite1 >= image_thresholded.cols / 2)
		centreLane = (centreWhite1) / 2;
	if (centreWhite1 < image_thresholded.cols / 2)
		centreLane = (centreWhite1 + image_thresholded.cols) / 2;
	cout << "CentreLane:" << centreLane << endl;
	//Put large dot in average of white lines
	Point centre;
	centre.x = centreLane;
	centre.y = row;
	circle(image_direction, centre, 20, CV_RGB(250, 100, 255), 1, 8, 0);
	return centreLane;
}

//Improved version of find centre when exactly 2 lanes have been detected
int findcentre2(int row) {
	int Mit;
	int countWhite = 0;
	int centreWhite1 = 0;
	int centreWhite2 = 0;
	int centreLane;
	int transition = 0;
	int lastValue = (image_thresholded.at<uchar>(row, 0)) % 2;
	for (int i = 0; i < image_thresholded.cols; i++) {
		Mit = (image_thresholded.at<uchar>(row, i)) % 2; // had problems with data type of binary image. Modulo works to get either 1 or 0
		if (lastValue != Mit) {

			if (transition == 0) {
				countWhite = i;
				centreWhite1 = countWhite;
			}

			else if (transition == 1) {
				centreWhite1 = (countWhite + i) / 2;
				countWhite = 0;
			} else if (transition == 2) {
				countWhite = i;
				centreWhite2 = countWhite;
			} else if (transition == 3) {
				centreWhite2 = (countWhite + i) / 2;
				countWhite = 0;
			}
			transition++;
		}
		lastValue = Mit;
	}
	centreLane = (centreWhite1 + centreWhite2) / 2;
	cout << "centreWhite1: " << centreWhite1 << endl;
	cout << "centreWhite2: " << centreWhite2 << endl;
	cout << "CentreLane:" << centreLane << endl;
	//Put large dot in average of white lines
	Point centre;
	centre.x = centreLane;
	centre.y = row;
	circle(image_direction, centre, 20, CV_RGB(250, 100, 255), 1, 8, 0);
	return centreLane;
}

// Counts the number of lines seen across the image
int countLines(int row, bool mode) {
	int Mit;
	int lineCount = 0;
	int transitionCount = 0;
	int lastValue = (image_thresholded.at<uchar>(row, 0)) % 2;
	for (int i = 0; i < image_thresholded.cols; i++) {
		Mit = (image_thresholded.at<uchar>(row, i)) % 2;
		if (Mit != lastValue) {
			transitionCount++;
		}
		lastValue = Mit;
	}
	lineCount = ceil(transitionCount / 2.0);
	cout << "Number of Transitions: " << transitionCount << endl;
	cout << "Number of Lines: " << lineCount << endl;
	if (mode == 0)
		return lineCount;
	if (mode == 1)
		return transitionCount;
}

/**
 * This is similar to the implementation of Robert Laganière.
 * See his book: OpenCV 2 Computer Vision Application Programming Cookbook.
 */
cv::Mat showHistogram(const cv::Mat &inImage) {

	cv::MatND hist;
	// For a gray scale [0:255] we have 256 bins
	const int bins[1] = { 256 };
	const float hranges[2] = { 0.0, 255.0 };
	const float* ranges[1] = { hranges };
	const int channels[1] = { 1 };

	cv::calcHist(&inImage, 1, // histogram from 1 image only
			channels, cv::Mat(), // no mask is used
			hist, // the output histogram
			1, // 1D histogram
			bins, ranges // pixel value ranges
			);

	// Get min and max bin values
	double maxVal = 0;
	double minVal = 0;
	cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);
	// The image to display the histogram
	cv::Mat histImg(bins[0], bins[0], CV_8U, cv::Scalar(255));

	// Map the highest point to 95% of the histogram height to leave some
	// empty space at the top
	const int histHeight = bins[0];
	const int maxHeight = 0.95 * histHeight;

	cv::Mat_<float>::iterator it = hist.begin<float>();
	cv::Mat_<float>::iterator itend = hist.end<float>();

	int barPosition = 0;
	for (; it != itend; ++it) {
		float histValue = (*it);
		int barHeight = (histValue * maxHeight) / maxVal;
		cv::line(histImg,
				// start the line from the bottom, and go up based on the barHeight
				// Remember the (0,0) is the top left corner
				cv::Point(barPosition, histHeight),
				cv::Point(barPosition, histHeight - barHeight),
				cv::Scalar::all(0));
		barPosition++;
	}
	return histImg;
}

void dump(const Mat &mat, const char* fname) {
	ofstream filestream;
	filestream.open(fname);
	filestream << mat << endl << endl;
	filestream.close();
}

void showHSVHistograms(Mat image) {

	vector<Mat> channels;

	//cvtColor(image, image_histo, COLOR_RGB2HSV);
	image_histo = image;
	split(image_histo, channels);

	// And then if you like
	image_H = channels[0];
	image_S = channels[1];
	image_V = channels[2];

	// Calculate histograms of image
	//histogram_H = showHistogram2(image_H);
	histogram_H = showHistogram4(image_H, 180, 0);
	threshold(image_H, image_HThresh, 0, 255, THRESH_OTSU | THRESH_BINARY);

	//histogram_S = showHistogram3(image_S);
	histogram_S = showHistogram4(image_S, 255, 0);
	threshold(image_S, image_SThresh, 0, 255, THRESH_OTSU | THRESH_BINARY);

	histogram_V = showHistogram4(image_V, 255, 0);
	threshold(image_V, image_VThresh, 0, 255, THRESH_OTSU | THRESH_BINARY);
}

cv::Mat showHistogram2(const cv::Mat &inImage) {

	cv::MatND hist;
	// For a gray scale [0:255] we have 256 bins
	const int bins[1] = { 180 };
	const float hranges[2] = { 0.0, 180.0 };
	const float* ranges[1] = { hranges };
	const int channels[1] = { 0 };

	cv::calcHist(&inImage, 1, // histogram from 1 image only
			channels, cv::Mat(), // no mask is used
			hist, // the output histogram
			1, // 1D histogram
			bins, ranges // pixel value ranges
			);

	// Get min and max bin values
	double maxVal = 0;
	double minVal = 0;
	cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);
	// The image to display the histogram
	cv::Mat histImg(bins[0], bins[0], CV_8U, cv::Scalar(180));

	// Map the highest point to 95% of the histogram height to leave some
	// empty space at the top
	const int histHeight = bins[0];
	const int maxHeight = 0.95 * histHeight;

	cv::Mat_<float>::iterator it = hist.begin<float>();
	cv::Mat_<float>::iterator itend = hist.end<float>();

	int barPosition = 0;
	for (; it != itend; ++it) {
		float histValue = (*it);
		int barHeight = (histValue * maxHeight) / maxVal;
		cv::line(histImg,
				// start the line from the bottom, and go up based on the barHeight
				// Remember the (0,0) is the top left corner
				cv::Point(barPosition, histHeight),
				cv::Point(barPosition, histHeight - barHeight),
				cv::Scalar::all(0));
		barPosition++;
	}
	return histImg;
}

cv::Mat showHistogram3(const cv::Mat &inImage) {

	cv::MatND hist;
	// For a gray scale [0:255] we have 256 bins
	const int bins[1] = { 256 };
	const float hranges[2] = { 0.0, 255.0 };
	const float* ranges[1] = { hranges };
	const int channels[1] = { 0 };

	cv::calcHist(&inImage, 1, // histogram from 1 image only
			channels, cv::Mat(), // no mask is used
			hist, // the output histogram
			1, // 1D histogram
			bins, ranges // pixel value ranges
			);

	// Get min and max bin values
	double maxVal = 0;
	double minVal = 0;
	cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);
	// The image to display the histogram
	cv::Mat histImg(bins[0], bins[0], CV_8U, cv::Scalar(255));

	// Map the highest point to 95% of the histogram height to leave some
	// empty space at the top
	const int histHeight = bins[0];
	const int maxHeight = 0.95 * histHeight;

	cv::Mat_<float>::iterator it = hist.begin<float>();
	cv::Mat_<float>::iterator itend = hist.end<float>();

	int barPosition = 0;
	for (; it != itend; ++it) {
		float histValue = (*it);
		int barHeight = (histValue * maxHeight) / maxVal;
		cv::line(histImg,
				// start the line from the bottom, and go up based on the barHeight
				// Remember the (0,0) is the top left corner
				cv::Point(barPosition, histHeight),
				cv::Point(barPosition, histHeight - barHeight),
				cv::Scalar::all(0));
		barPosition++;
	}
	return histImg;
}

cv::Mat showHistogram4(const Mat &inImage, int numBins, int numChannels) {

	cv::MatND hist;
	// For a gray scale [0:255] we have 256 bins
	const int bins[1] = { numBins };
	const float hranges[2] = { 0.0, numBins / 1.0 };
	const float* ranges[1] = { hranges };
	const int channels[1] = { numChannels };

	cv::calcHist(&inImage, 1, // histogram from 1 image only
			channels, cv::Mat(), // no mask is used
			hist, // the output histogram
			1, // 1D histogram
			bins, ranges // pixel value ranges
			);

	// Get min and max bin values
	double maxVal = 0;
	double minVal = 0;
	cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);
	// The image to display the histogram
	cv::Mat histImg(bins[0], bins[0], CV_8U, cv::Scalar(255));

	// Map the highest point to 95% of the histogram height to leave some
	// empty space at the top
	const int histHeight = bins[0];
	const int maxHeight = 0.95 * histHeight;

	cv::Mat_<float>::iterator it = hist.begin<float>();
	cv::Mat_<float>::iterator itend = hist.end<float>();

	int barPosition = 0;
	for (; it != itend; ++it) {
		float histValue = (*it);
		int barHeight = (histValue * maxHeight) / maxVal;
		cv::line(histImg,
				// start the line from the bottom, and go up based on the barHeight
				// Remember the (0,0) is the top left corner
				cv::Point(barPosition, histHeight),
				cv::Point(barPosition, histHeight - barHeight),
				cv::Scalar::all(0));
		barPosition++;
	}
	return histImg;
}

