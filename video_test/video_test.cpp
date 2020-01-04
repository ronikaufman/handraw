#include <iostream>
#include <math.h>
#include <list>
#include <stdlib.h>

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;


// ********** global variables **********


// definition of the drawing rectangle
float cap_region_x_begin = 0.5;
float cap_region_y_end = 0.8;

// delay for update of the number of fingers
int nbFrames = 10;

// storage for the number of shown fingers in the last nbFrames frames
vector<int> fingers(nbFrames, -1);

// drawn lines
vector<vector<Point> > drawn(0);
// current line
vector<Point> currentLine(0);
// erased points
vector<Point> eraser(0);


// ********** functions **********


// updates the background subtractor and returns the extracted hand frame

Mat removeBackground(Mat cameraFrame, Ptr<BackgroundSubtractor> pBackSub) {
  Rect ROI(Point(int(cap_region_x_begin * cameraFrame.size[1]), 0),
           Point(cameraFrame.size[1],
                 int(cap_region_y_end * cameraFrame.size[0])));
	Mat frame(cameraFrame, ROI);

  Mat fgMask, res;
  pBackSub->apply(frame, fgMask, -1.0);
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

// returns convexity defects

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

      int depthThresh = 8000;
      if (angle <= 3.14159/1.5 && defects[i].val[3] > depthThresh) {
        count++;
      }
    }
  }
  return (count + 1);
}

// show convexity hulls (computes it again)
// only for testing

void showConvexityHull(vector<vector<Point> > contours,
                       int maxContour,
                       vector<Vec4i> hierarchy,
                       Mat& Image) {

  vector<vector<Point> >hull(contours.size());
  for (int i = 0; i < contours.size(); i++) {
    convexHull(Mat(contours[i]), hull[i], false);
  }
  drawContours(Image, hull, maxContour, CV_RGB(0, 0, 255), 2, 8, hierarchy);
}

// show convexity getDefects
// only for testing

void showConvexityDefects(vector<Vec4i> defects,
                          vector<Point> contour,
                          Mat& Image) {

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

// returns the tip of the finger
// (the furthest from the contour's centroid and above it)

Point getFingertip(vector<Vec4i> defects,
                   vector<Point> contour,
                   Mat& Image,
                   int nFingers) {

  Moments m = moments(contour);
	Point centroid(m.m10/m.m00, m.m01/m.m00);

  Point max;
  double maxDist = 0;

  for (int i = 0; i < defects.size(); i++) {
    int start = defects[i].val[0];
    Point pt(contour[start]);
    double dist = norm(pt - centroid);
    if (dist > maxDist && pt.y < centroid.y) {
      max = pt;
      maxDist = dist;
    }
  }

  max.x = max.x + Image.size[1] - int(cap_region_x_begin * Image.size[1]);
	return max;
}

// deletes the point p from defects

void erasePoint(vector<Vec4i> defects,
                Point p,
                vector<Point> contour,
                Mat& Image) {
  for (int i = 0; i < defects.size(); i++) {
    int start = defects[i].val[0];
    Point pt(contour[start]);
    pt.x = pt.x + Image.size[1] - int(cap_region_x_begin * Image.size[1]);
    if (pt == p) {
      defects.erase(defects.begin() + i);
      break;
    }
  }
}

// returns the number of fingers shown
// (an average on the last nbFrames, actually)

float averageFinger(vector<Point> contour,
                    vector<Vec4i> defects,
                    int nFingers) {
	int i = 0;
	while ((i < nbFrames) && (fingers[i] != -1)) {
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

// erases the point p in drawn

void eraseWith(Point p) {
  float delta = 5; // proximity tolerance
  int i = 0;
  bool cut = false;
  while (i < drawn.size()) {
    cut = false;
    vector<Point> line = drawn[i];

    for (int j = 0; j < line.size(); j++) {
      if (norm(line[j] - p) < delta) { // delete line[j]
        if (j > 0) {
          vector<Point> secondHalf(0);
          for (int k = j + 1; k < line.size(); k++) {
            secondHalf.push_back(line[k]);
          }
          drawn.push_back(secondHalf);
        }

        if (j < line.size() - 1) {
          vector<Point> firstHalf(0);
          for (int k = 0; k < j; k++) {
            firstHalf.push_back(line[k]);
          }
          drawn.push_back(firstHalf);
        }

        drawn.erase(drawn.begin() + i);

        cut = true;
        break;
      }
    }

    if (!cut) {
      i++;
    }
  }
}


// ********** main **********


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

  // Create Background Subtractor object
  Ptr<BackgroundSubtractor> pBackSub;
  int history = 500;
  double varThreshold = 400;
  //pBackSub = createBackgroundSubtractorMOG2(history, varThreshold);
  pBackSub = createBackgroundSubtractorKNN(history, varThreshold,true);

  Mat cameraFrame, hand;

  int countFrames = 0;

  // Unconditional loop
  while (true) {
    stream1.read(cameraFrame);
    if (cameraFrame.empty()) {
      cout << "end of video" << endl;
      break;
    }

  	flip(cameraFrame, cameraFrame, 1);

  	// Create drawing zone rectangle
  	rectangle(cameraFrame,
              Point(int(cap_region_x_begin * cameraFrame.size[1]), 0),
              Point(cameraFrame.size[1],
                    int(cap_region_y_end * cameraFrame.size[0])),
              Scalar(0, 0, 0),
              3);

    // Identify hand
  	hand = removeBackground(cameraFrame, pBackSub);
  	//imshow("only hand", hand); // for testing

  	// Create Threshold image
  	Mat threshImage;
  	double thresholdValue = 100;
  	threshold(hand, threshImage, thresholdValue, 255, THRESH_BINARY);
  	dilate(threshImage,
           threshImage,
           Mat(),
           Point(-1, -1),
           2,
           BORDER_CONSTANT,
           morphologyDefaultBorderValue());
  	//imshow("threshold", threshImage); // for testing

  	// Create contours & defects
  	vector<vector<Point> > contours;
  	vector<Vec4i> hierarchy;
  	findContours(threshImage,
                 contours,
                 hierarchy,
                 RETR_LIST,
                 CHAIN_APPROX_SIMPLE);

    // for testing:
  	//Mat contoursImage = Mat::zeros(threshImage.rows, threshImage.cols, CV_8UC3);
  	//Scalar color(255, 0, 0);
  	//drawContours(contoursImage, contours, -1, color, 1, 8, hierarchy);

  	int maxContour = getMaxContour(contours);
    float average = -1;

  	if (maxContour != -1) {

    	vector<Vec4i> defects = getDefects(contours[maxContour]);
    	//showConvexityHull(contours, maxContour, hierarchy, contoursImage);
    	//showConvexityDefects(defects, contours[maxContour], contoursImage);

    	// Count fingers
    	int nFingers = countFingers(contours[maxContour], defects);

    	// Calculate average number of fingers (nbFrames frames)
    	average = averageFinger(contours[maxContour], defects, nFingers);

    	// Actions
    	if (defects.size() > 0) {
    		if (average > 0.6 && average <= 1.7) {
          // One finger up -> draw with the point
    			Point point = getFingertip(defects,
                                     contours[maxContour],
                                     cameraFrame,
                                     nFingers);
    			currentLine.push_back(point);
    		} else if (!currentLine.empty()) {
          // begin a new line
          drawn.push_back(currentLine);
          currentLine.clear();
        }

        if (average > 2.8 && average <= 3.6) {
          // Three fingers up -> erase with the one on the left
  				Point point1 = getFingertip(defects,
  									                  contours[maxContour],
  									                  cameraFrame,
  									                  nFingers);
          erasePoint(defects, point1, contours[maxContour], cameraFrame);
          Point point2 = getFingertip(defects,
                                      contours[maxContour],
         									            cameraFrame,
         									            nFingers);

          Point point;
          if (point1.x < point2.x) {
            point = point1;
          } else {
            point = point2;
          }

  				if (eraser.size() < 1) {
            eraser.push_back(point);
  				} else {
  					assert(!eraser.empty());
  					eraser.erase(eraser.begin());
            eraser.push_back(point);
  				}

          eraseWith(point);

      		if ((defects.size() > 0) && !eraser.empty()) {
      			for (int i = 0; i < eraser.size(); i++) {
    				  circle(cameraFrame, eraser[i], 6, CV_RGB(255, 255,255), 3, 8);
      			}
      		}
        }

        if (average > 4.3) {
          // Five fingers up -> erase
          drawn.clear();
        }
    	}

    	// Plot
    	if ((defects.size() > 0) &&  !currentLine.empty() ) {
    		for (int i = 0; i < currentLine.size(); i++) {
    			circle(cameraFrame, currentLine[i], 1, CV_RGB(0, 0, 0), 3, 8);
    		}
    	}

      polylines(cameraFrame, currentLine, false, CV_RGB(0, 0, 0), 3, 8, 0);
      for (int i = 0; i < drawn.size(); i++) {
        polylines(cameraFrame, drawn[i], false, CV_RGB(0, 0, 0), 3, 8, 0);
      }

    }

  	//imshow("contours + defects", contoursImage);
  	imshow("cam", cameraFrame);

  	if (countFrames % nbFrames == 0) {
  		cout << "Average: " << average << endl;
  	}

    countFrames++;

  	if (waitKey(30) >= 0) break;

  } // end of while

  // Close all the windows
  destroyAllWindows();
  return 0;
} // end of main
