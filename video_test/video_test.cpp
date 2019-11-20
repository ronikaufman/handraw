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

int calculateFingers(vector<vector<Point> > contours) {
	vector<vector<Point> > hull(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		convexHull(Mat(contours[i]), hull[i], false);
	}

	int s = hull.size();

//	if (size.height > 3) {
//		vector<int> defects;
/*		convexityDefects(res, hull,defects);
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
	}

*/
	return 0;
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
   

	//Mat hull;
	//convexHull(contoursImage, hull);
	//drawContours(contoursImage, hull, -1, color, 1, 8, hierarchy);
	
	

/*	// create a blank image (black image)
	Mat drawing = Mat::zeros(threshImage.rows, threshImage.cols, CV_8UC3);

	for (int i = 0; i < contours.size(); i++){
		Scalar color_contours = Scalar(0, 255, 0); // green - color for contours
		Scalar color = Scalar(255, 0, 0); // blue - color for convex hull
		// draw ith contour
		drawContours(drawing, contours, i, color_contours, 1, 8, vector<Vec4i>(), 0, Point());
		// draw ith convex hull
		drawContours(drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point());
  }

//	imshow("contours", contoursImage);
	imshow("hull", drawing);
*/
    if (waitKey(30) >= 0)
      break;
  }

  // Closes all the windows
  destroyAllWindows();
  return 0;
}
