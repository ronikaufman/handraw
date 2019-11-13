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

Mat removeBackground(Mat frame, Ptr<BackgroundSubtractor> pBackSub) {
  Mat fgMask, res;
  pBackSub->apply(frame, fgMask);
  bitwise_and(frame, frame, res, fgMask);
  int blurValue = 41;
  cvtColor(res, res, COLOR_BGR2GRAY);
  GaussianBlur(res, res, Size(blurValue, blurValue), 1, 1);
  return res;
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
  double varThreshold = 100; // CHANGE?
  pBackSub = createBackgroundSubtractorMOG2(history, varThreshold);
  //pBackSub = createBackgroundSubtractorKNN(history, 400);

  Mat cameraFrame, hand;

  //unconditional loop
  while (true) {
    stream1.read(cameraFrame);
    if (cameraFrame.empty()) {
      cout << "end of video" << endl;
      break;
    }

    imshow("cam", cameraFrame);

    hand = removeBackground(cameraFrame, pBackSub);
    imshow("only hand", hand);
    Mat threshImage;
    double thresholdValue = 100; // CHANGE?
    threshold(hand, threshImage, thresholdValue, 255, THRESH_BINARY);
    imshow("threshold", threshImage);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(threshImage, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    Mat contoursImage = Mat::zeros(threshImage.rows, threshImage.cols, CV_8UC3);
    Scalar color(255, 0, 0);
    drawContours(contoursImage, contours, -1, color, 1, 8, hierarchy);
    imshow("contours", contoursImage);

    if (waitKey(30) >= 0)
      break;
  }

  // Closes all the windows
  destroyAllWindows();
  return 0;
}
