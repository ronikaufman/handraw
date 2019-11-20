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
using namespace cv;
using namespace std;


float cap_region_x_begin = 0.5;
float cap_region_y_end = 0.8;

int isBgCaptured = 0;

Mat removeBackground(Mat frame, Ptr<BackgroundSubtractor> pBackSub) {
  Mat fgMask, res;
  pBackSub->apply(frame, fgMask);
  bitwise_and(frame, frame, res, fgMask);
  int blurValue = 41;
  cvtColor(res, res, COLOR_BGR2GRAY);
  GaussianBlur(res, res, Size(blurValue, blurValue), 1, 1);
  return res;
}

// DEPRECATED :(, DON'T USE!
int calculateFingers(vector<vector<Point> > contours) {
  vector<vector<Point> > hull(contours.size());
  vector<vector<int> > hullsI(contours.size()); // Indices to contour points
  vector<vector<Vec4i> > defects(contours.size());

  for (int i = 0; i < contours.size(); i++) {
    convexHull(contours[i], hull[i], false);
    convexHull(contours[i], hullsI[i], false);
    if (hullsI[i].size() > 3) {
        convexityDefects(contours[i], hullsI[i], defects[i]);
    }
  }

  for (int i = 0; i < defects.size(); i++) {
    //cout << defects[i] << endl;
  }


  /*
	vector<vector<Point> > hull(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		convexHull(Mat(contours[i]), hull[i], false);
	}

	int s = hull.size();

	if (s > 3) {
		//vector<int> defects;
    vector<vector<Vec4i>> defects;
		convexityDefects(contours, hull, defects);

		if (true) {
			int cnt = 0;
			for (int i = 0; i <= defects.size(); i = i + 1) {
				int s;
				int e;
				int f;
				int d;
				s = defects[i];
				cout << s << endl;
			}
		}
	}*/

	return 0;
}

// returns index of max contour

int getMaxContour(vector<vector<Point> > contours) {
  int index = -1;
  int maxSize = 0;
  for (int i = 0; i < contours.size(); i++) {
    if (contours[i].size() > maxSize) {
      maxSize = contours[i].size();
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

      double a = sqrt(pow((ptEnd.x - ptStart.x), 2) + pow((ptEnd.y - ptStart.y), 2));
      double b = sqrt(pow((ptFar.x - ptStart.x), 2) + pow((ptFar.y - ptStart.y), 2));
      double c = sqrt(pow((ptEnd.x - ptFar.x), 2) + pow((ptEnd.y - ptFar.y), 2));
      double angle = acos((b * b + c * c - a * a) / (2 * b * c));

      int depthThresh = 10000; //CHANGE?
      if (angle <= M_PI/2 && defects[i].val[3] > depthThresh) {
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

  //create Background Subtractor objects
  Ptr<BackgroundSubtractor> pBackSub;
  int history = 500; //CHANGE?
  double varThreshold = 400; // CHANGE?
  //pBackSub = createBackgroundSubtractorMOG2(history, varThreshold);
  pBackSub = createBackgroundSubtractorKNN(history, varThreshold,true);

  Mat cameraFrame, hand;

  //unconditional loop
  while (true) {
    stream1.read(cameraFrame);
    if (cameraFrame.empty()) {
      cout << "end of video" << endl;
      break;
    }

	// Create rectangle
	flip(cameraFrame, cameraFrame, 1);
	rectangle(cameraFrame, Point(int(cap_region_x_begin * cameraFrame.size[1]), 0), Point(cameraFrame.size[1], int(cap_region_y_end * cameraFrame.size[0])), (0, 0, 0));

  imshow("cam", cameraFrame);

	// SmallFrame
	Rect ROI(Point(int(cap_region_x_begin * cameraFrame.size[1]), 0), Point(cameraFrame.size[1], int(cap_region_y_end * cameraFrame.size[0])));
	Mat smallFrame(cameraFrame, ROI);

	hand = removeBackground(smallFrame, pBackSub);

  imshow("only hand", hand);
  Mat threshImage;
  double thresholdValue = 50; // CHANGE?
  threshold(hand, threshImage, thresholdValue, 255, THRESH_BINARY);
  imshow("threshold", threshImage);

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

  imshow("contours + defects", contoursImage);

  int nFingers = countFingers(contours[maxContour], defects);
  cout << nFingers << endl;

	//Mat hull;
	//convexHull(contoursImage, hull);
	//drawContours(contoursImage, hull, -1, color, 1, 8, hierarchy);


  /*
  // create a blank image (black image)
	Mat drawing = Mat::zeros(threshImage.rows, threshImage.cols, CV_8UC3);
	for (int i = 0; i < contours.size(); i++){
		Scalar color_contours = Scalar(0, 255, 0); // green - color for contours
		Scalar color = Scalar(255, 0, 0); // blue - color for convex hull
		// draw ith contour
		drawContours(drawing, contours, i, color_contours, 1, 8, vector<Vec4i>(), 0, Point());
		// draw ith convex hull
		drawContours(drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point());
  }
  //imshow("contours", contoursImage);
	imshow("hull", drawing);
  */

  if (waitKey(30) >= 0)
    break;
  }

  // Closes all the windows
  destroyAllWindows();
  return 0;
}
