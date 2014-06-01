
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <string>
//---VISION-----------
#include <sstream>
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
//----END VISION---- <- AM WORKING ON CODE INDEPENDENT OF ROS, THIS WILL HELP ME COPY IN UPDATES
using namespace cv; 
using namespace std; 
using namespace ros; 
//---VISION---------------
//Function Declarations (Don't worry will get rid of a lot of these)
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
//--END VISION------------------------
//global strings for ros node and topic names
static const string NODE_NAME = "LaneDetector";
static const string PUBLISH_TOPIC = "vision_vel";
const int MSG_QUEUE_SIZE = 20;

double steering = 1;
double throttle = 2;
//---VISION---------------------
Mat image, image_grey, image_filter, image_thresholded, image_canny, image_blur,
		image_blur2, image_direction;
Mat image_H, image_S, image_V, image_histo, image_HThresh, image_SThresh,
		image_VThresh;
Mat histogram_H, histogram_S, histogram_V;
int const threshold_value = 180;
int blur_value = 2;
int const max_BINARY_value = 255;
// --END VISION -------

int main(int argc, char **argv)
{
  // ----ROS------
  init(argc, argv, NODE_NAME);
  NodeHandle n;
  Publisher chatter_pub = n.advertise<geometry_msgs::Twist>(PUBLISH_TOPIC, MSG_QUEUE_SIZE);

  ros::Rate loop_rate(10);

  int count = 0;
  geometry_msgs::Twist twist;
twist.linear.x = 0;
twist.linear.y = 0;
twist.linear.z = 0;
twist.angular.x = 0;
twist.angular.y = 0;
twist.angular.z = 0;
//---END ROS

//----VISION--------
//Initialize camera
	VideoCapture cap(0); // open the default camera
	cout<<"youououou";
	//VideoCapture cap("sample-course.avi");
	if (!cap.isOpened())
	{
		cout<<" Camera not opened";
		return -1;
	}
//---END VISION

  while (ros::ok())
  {

    std_msgs::String msg;
    std::stringstream ss;//TODO: delete?
    ss << "Twist Message is being output " << count;//TODO: delete?
    msg.data = ss.str();//TODO: delete?

//----VISION-----
  
   
   //Read image
    cap >> image; //from video
 
	if (!cap.read(image)) //if not success, break loop
	 {
	   cout << "Cannot read a frame from video stream" << endl;
	  break;
	  }
	
	cout<<"Image Read"<<endl;
	//image = imread("course.jpg"); // from picture

		filterImage();

		detectLines();

		//getDirection();

		displayWindows();

		// Wait for user input
		//waitKey(0); //use waitKey(0); to enter "debugging" mode
		if (waitKey(1) == 27)
			break; // Wait for one ms, break if escape is pressed
	

    chatter_pub.publish(twist);
   ROS_INFO("Vision Published: throttle - %f, steering - %f",throttle,steering);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
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
}

void getDirection(void) {

	int rows2Check = 2;
	int distanceBetweenRows = image.rows / 10;
	int const startRow = image.rows / 2 + distanceBetweenRows; //TODO: adjust for camera angle
	int row = startRow;

	Point points[4][rows2Check];
	int transitions[2];

	int lastValue = (image_thresholded.at<uchar>(row, 0)) % 2;
	int currentValue = 0;

	image_direction = image_thresholded.clone();

	//Check how many transitions occur in each row and put into array
	for (int i = 0; i < rows2Check; i++) {
		transitions[i] = countLines(row, 1);
		row = row - distanceBetweenRows;
		//cout << "Row: " << i << endl;
		//cout << "Transitions: " << transitions[i] << endl;
	}
	row = startRow;

	/* Categorize each transition
	 1. left side of left line
	 2. right side of left line
	 3. left side of right line
	 4. right side of right line
	 if only one line is visible it will be placed as a left line for now
	 */

	for (int i = 0; i < rows2Check; i++) {
		lastValue = (image_thresholded.at<uchar>(row, 0)) % 2;
		int transitionCount = 0;
		int left = 0;
		Point centre;

		if (transitions[i] == 0) {
			cout << "Error: No lines detected" << endl;
			//TODO: send out message with low confidence levels
		} else if (transitions[i] > 4) {
			cout << "Error: More than 2 lines detected" << endl;
		} else {
			for (int f = 0; f < image_thresholded.cols; f++) {
				currentValue = image_thresholded.at<uchar>(row, f) % 2;
				if (currentValue != lastValue) {
					cout << "Transition at x = " << f << ", y = " << row
							<< endl;
					centre.x = f;
					centre.y = row;
					circle(image_direction, centre, 5, CV_RGB(250, 100, 255), 1,8, 0);
					cout << "Current Value: " << currentValue << " Last Value"
							<< lastValue << endl;
				}
				//Here comes the fun!!!! Value of black is 1 and value of white is 0
				// 1 transition
				if (transitions[i] == 1) {
					if ((lastValue == 0) && (currentValue == 1))
						points[1][i] = centre;
					else if ((lastValue == 1) && (currentValue == 0))
						points[2][i] = centre;
				} else if (transitions[i] == 2) // this part is still iffy
						{
					if (left == 0) {
						if ((lastValue == 0) && (currentValue == 1))
							points[1][i] = centre; //RoL
						else if ((lastValue == 1) && (currentValue == 0))
							points[0][i] = centre; //LoL
						left = 1;
					} else if (left == 1) {
						if ((lastValue == 0) && (currentValue == 1))
							points[1][i] = centre; // RoL
						else if ((lastValue == 1) && (currentValue == 0))
							points[2][i] = centre; //LoR
					}
				} else if (transitions[i] == 3) {
					if (left == 0) {
						if ((lastValue == 0) && (currentValue == 1))
							points[1][i] = centre; // RoL
						else if ((lastValue == 1) && (currentValue == 0))
							points[0][i] = centre; //LoL
						left = 1;
					} else if (left == 1) {
						if ((lastValue == 0) && (currentValue == 1))
							points[1][i] = centre; // RoL
						else if ((lastValue == 1) && (currentValue == 0))
							points[2][i] = centre; //LoR
						left = 2;
					} else if (left == 2) {
						if ((lastValue == 0) && (currentValue == 1))
							points[3][i] = centre; // RoL
						else if ((lastValue == 1) && (currentValue == 0))
							points[2][i] = centre; //LoR
					}
				} else if (transitions[i] == 4) {
						if (left == 0) {
							if ((lastValue == 1) && (currentValue == 0))
								points[0][i] = centre; //LoL
							left = 1;
						} else if (left == 1) {
							if ((lastValue == 0) && (currentValue == 1))
								points[1][i] = centre; // RoL
							left = 2;
						} else if (left == 2) {
							if ((lastValue == 1) && (currentValue == 0))
								points[2][i] = centre; // RoL
							left = 3;
						} else if (left == 3) {
							if ((lastValue == 0) && (currentValue == 1))
								points[3][i] = centre; // RoL
						}
					}

					lastValue = currentValue;
				}
				drawLine(row);
				row = row - distanceBetweenRows;
			}
		}
		// Ok, now we have the matrix lets do some calculatin'
	}


	/*
	 float(calculateDirection){

	 }
	 */
	float calculateDirection1(void) {
		int iterations = 2;
		int distanceBetweenRows = image.rows / 10;
		int startRow = image.rows / 2; //TODO: adjust for camera angle
		int row = startRow -	 distanceBetweenRows;

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
		int dx = 0;
		int dy = 0;
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
				circle(image_direction, centre, 5, CV_RGB(250, 100, 255), 1, 8,
						0);
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


