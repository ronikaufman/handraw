/*#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/video/background_segm.hpp>*/

#include <opencv2/opencv.hpp>

#include <iostream>
#include <math.h>
#include <list>
#include <stdlib.h>

using namespace cv;
using namespace std;

// Definition of the Region of Interest
float cap_region_x_begin = 0.5;
float cap_region_y_end = 0.8;

int isBgCaptured = 0;

// Definition of the delay for update
int nbFrames = 15;
int show = 0;
vector<int> fingers(nbFrames,-1);
vector<Point> drawn(nbFrames);


Mat removeBackground(Mat frame, Ptr<BackgroundSubtractor> pBackSub) {
  Mat fgMask, res;
  pBackSub->apply(frame, fgMask);
  bitwise_and(frame, frame, res, fgMask);
  int blurValue = 41;
  cvtColor(res, res, COLOR_BGR2GRAY);
  GaussianBlur(res, res, Size(blurValue, blurValue), 1, 1);
  return res;
}

// returns index of contour of max area

int getMaxContour(vector<vector<Point> > contours) {
  int index = -1;
  int maxSize = 0;
  for (int i = 0; i < contours.size(); i++) {
    if (contourArea(contours[i]) > maxSize) {
      maxSize = contourArea(contours[i]);
      index = i;
    }
  }
  return index;
}

// get convexity convexityDefects

vector<Vec4i> getDefects(vector<Point> contour) {
  vector<Point> hull;
  vector<int> hullsI; // indices to contour points
  vector<Vec4i> defects;

  convexHull(contour, hull, false);
  convexHull(contour, hullsI, false); // indices of contour points
  if (hullsI.size() > 3) {
    convexityDefects(contour, hullsI, defects);
  }

  return defects;
}

// counts the number of calculateFingers

int countFingers(vector<Point> contour, vector<Vec4i> defects) {
  int count = 0;
  int dSize = defects.size();
  if (dSize > 0) {
    for (int i = 0; i < dSize; i++) {
      int start = defects[i].val[0];
      Point ptStart(contour[start]);

      int end = defects[i].val[1];
      Point ptEnd(contour[end]);

      int far = defects[i].val[2];
      Point ptFar(contour[far]);

      double a = sqrt(pow((ptEnd.x - ptStart.x), 2)
                    + pow((ptEnd.y - ptStart.y), 2));
      double b = sqrt(pow((ptFar.x - ptStart.x), 2)
                    + pow((ptFar.y - ptStart.y), 2));
      double c = sqrt(pow((ptEnd.x - ptFar.x), 2)
                    + pow((ptEnd.y - ptFar.y), 2));
      double angle = acos((b * b + c * c - a * a) / (2 * b * c));

      int depthThresh = 5000; //CHANGE?
      if (angle <= 3.14159/2 && defects[i].val[3] > depthThresh) {
        count++;
      }
    }
  }
  return count;
}

// show convexity hulls (computes it again)

void showConvexityHull(vector<vector<Point> > contours, int maxContour, vector<Vec4i> hierarchy, Mat& Image) {
  vector<vector<Point> >hull(contours.size());
  for (int i = 0; i < contours.size(); i++) {
    convexHull(Mat(contours[i]), hull[i], false);
  }
  drawContours(Image, hull, maxContour, CV_RGB(0, 0, 255), 2, 8, hierarchy);
}


// show convexity getDefects

void showConvexityDefects(vector<Vec4i> defects, vector<Point> contour, Mat& Image) {
  for (int i = 0; i < defects.size(); i++) {
    int start = defects[i].val[0];
    Point ptStart(contour[start]);

    int end = defects[i].val[1];
    Point ptEnd(contour[end]);

    int far = defects[i].val[2];
    Point ptFar(contour[far]);

    circle(Image, ptStart, 5, CV_RGB(255,0,0), 2, 8);
    circle(Image, ptEnd, 5, CV_RGB(255, 255, 0), 2, 8);
    circle(Image, ptFar, 5, CV_RGB(0,0,255), 2, 8);
  }
}

// Returns the tip of the finger (the furthest from the contour's centroid)
Point getFingertip(vector<Vec4i> defects, vector<Point> contour, Mat& Image, int nFingers) {
  Moments m = moments(contour);
	Point centroid(m.m10/m.m00, m.m01/m.m00);

	Point max(contour[defects[0].val[0]]);
  double maxDist = norm(max - centroid);

  /*
	max.x = max.x + Image.size[1] - int(cap_region_x_begin * Image.size[1]);
	for (int i = 0; i < defects.size(); i++) {
		int start = defects[i].val[0];
		Point ptStart(contour[start]);
		ptStart.x = ptStart.x + Image.size[1] - int(cap_region_x_begin * Image.size[1]);
		if (max.y > ptStart.y) {
			max = ptStart;
		}
	}
  */

  for (int i = 1; i < defects.size(); i++) {
    int start = defects[i].val[0];
    Point pt(contour[start]);
    double dist = norm(pt - centroid);
    if (dist > maxDist) {
      max = pt;
      maxDist = dist;
    }
  }

  max.x = max.x + Image.size[1] - int(cap_region_x_begin * Image.size[1]);
	return max;
}

float averageFinger(vector<Point> contour, vector<Vec4i> defects, int nFingers) {
	int i = 0;
	while ((fingers[i] != -1) && (i < nbFrames)) {
		i++;
	}
	if (i < nbFrames) {
		fingers[i] = nFingers;
	}
	else {
		assert(!fingers.empty());
		fingers.erase(fingers.begin());
		fingers.push_back(nFingers);
	}
	float average = 0;
	for (int i = 0; i < nbFrames; i++) {
		if (fingers[i] != -1) {
			average = average + fingers[i];
		}
	}
	average = average / nbFrames*1.0;
	return average;
}

int main(int argc, char** argv) {
  VideoCapture stream1;
  if (argc == 1) {
    cout << "no filename given, using webcam" << endl;
    stream1 = VideoCapture(0);
  } else {
    cout << "using file " << argv[1] << endl;
    stream1 = VideoCapture(argv[1]);
  }

  if (!stream1.isOpened()) { //check if video device has been initialised
    cout << "cannot open video file" << endl;
    return 0;
  }

  // create Background Subtractor object
  Ptr<BackgroundSubtractor> pBackSub;
  int history = 500; //CHANGE?
  double varThreshold = 400; // CHANGE?
  //pBackSub = createBackgroundSubtractorMOG2(history, varThreshold);
  pBackSub = createBackgroundSubtractorKNN(history, varThreshold,true);

  Mat cameraFrame, hand;

  // Unconditional loop
  while (true) {
    stream1.read(cameraFrame);
    if (cameraFrame.empty()) {
      cout << "end of video" << endl;
      break;
    }

	// Create rectangle
	flip(cameraFrame, cameraFrame, 1);
	rectangle(cameraFrame,
            Point(int(cap_region_x_begin * cameraFrame.size[1]), 0),
            Point(cameraFrame.size[1], int(cap_region_y_end * cameraFrame.size[0])),
            (0, 0, 0),
            5);

	// create smallFrame
	Rect ROI(Point(int(cap_region_x_begin * cameraFrame.size[1]), 0),
           Point(cameraFrame.size[1], int(cap_region_y_end * cameraFrame.size[0])));
	Mat smallFrame(cameraFrame, ROI);

	hand = removeBackground(smallFrame, pBackSub);

	// Create Threshold image
	imshow("only hand", hand);
	Mat threshImage;
	double thresholdValue = 40; // CHANGE?
	threshold(hand, threshImage, thresholdValue, 255, THRESH_BINARY);
	imshow("threshold", threshImage);

	// Create contours & defects
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(threshImage, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

	Mat contoursImage = Mat::zeros(threshImage.rows, threshImage.cols, CV_8UC3);
	Scalar color(255, 0, 0);
	drawContours(contoursImage, contours, -1, color, 1, 8, hierarchy);

	int maxContour = getMaxContour(contours);
	vector<Vec4i> defects = getDefects(contours[maxContour]);
	showConvexityHull(contours, maxContour, hierarchy, contoursImage);
	showConvexityDefects(defects, contours[maxContour], contoursImage);

	// Count fingers
	int nFingers = countFingers(contours[maxContour], defects);

	// Calculate average number of fingers (nbFrames frames)
	float average = averageFinger(contours[maxContour], defects, nFingers);

  float margin = 0.6;

	// Actions
	if (defects.size() > 0) {
		if (abs(average - 1) < margin) {
      // one finger up -> draw with the point
			Point point = getFingertip(defects, contours[maxContour], cameraFrame, nFingers);
			//circle(cameraFrame, point, 3, CV_RGB(0, 0, 0), 3, 8);
			if (drawn.size() < nbFrames) {
				drawn.push_back(point);
			}
			else {
				assert(!drawn.empty());
				drawn.erase(drawn.begin());
				drawn.push_back(point);
			}

			for (int i = 0; i < drawn.size(); i++) {
				circle(cameraFrame, drawn[i], 3, CV_RGB(0, 0, 0), 3, 8);
			}
		} else if (abs(average - 2) < margin) {
      // two fingers up -> erase with these fingers
      // TO DO
    } else if (abs(average - 5) < margin) {
      // five fingers up -> erase
      drawn.clear();
    }
	}

	// Plot
	imshow("contours + defects", contoursImage);
	imshow("cam", cameraFrame);

	//cout << nFingers << endl;

	if (show == 3*nbFrames) {
		show = 0;
		cout << "Average: " << average << endl;
	}
	else {
		show++;
	}

	if (waitKey(30) >= 0) break;

  }

  // Closes all the windows
  destroyAllWindows();
  return 0;
}
