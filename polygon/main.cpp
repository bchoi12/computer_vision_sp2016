#include <stdio.h>
#include <stdlib.h>

#include <iostream>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

bool visible(int xstart, int ystart, int xend, int yend, cv::Mat &freespace);
void swap(int &one, int &two);

int main(int argc, char** argv) {
  using namespace cv;

  Mat walls = imread("walls.ppm");
  Mat walls_gray = imread("walls.ppm", CV_LOAD_IMAGE_GRAYSCALE);
  Mat freespace = imread("freespace.ppm");
  Mat freespace_gray = imread("freespace.ppm", CV_LOAD_IMAGE_GRAYSCALE);

  std::vector<Vec4i> lines;

  // params: output, lines, resolution, resolution of theta, min number of intersections, min line length, max line gap
  HoughLinesP(walls_gray, lines, 1, CV_PI/180, 10, 5, 3);

  for(unsigned int i=0; i<lines.size(); ++i) {
    line(walls, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0, 255, 0));
    circle(walls, Point(lines[i][0], lines[i][1]), 2, Scalar(0, 0, 255));
    circle(walls, Point(lines[i][2], lines[i][3]), 2, Scalar(0, 0, 255));

    line(freespace, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0, 255, 0));
    circle(freespace, Point(lines[i][0], lines[i][1]), 2, Scalar(0, 0, 255));
    circle(freespace, Point(lines[i][2], lines[i][3]), 2, Scalar(0, 0, 255));
  }

  for(unsigned int i=0; i<lines.size(); ++i) {
    for(unsigned int j=i+1; j<lines.size(); ++j) {
      
      if (visible(lines[i][0], lines[i][1], lines[j][0], lines[j][1], freespace_gray)) {
	line(freespace, Point(lines[i][0], lines[i][1]), Point(lines[j][0], lines[j][1]), Scalar(255, 0, 0));
      }
      if (visible(lines[i][2], lines[i][3], lines[j][0], lines[j][1], freespace_gray)) {
	line(freespace, Point(lines[i][2], lines[i][3]), Point(lines[j][0], lines[j][1]), Scalar(255, 0, 0));
      }
      if (visible(lines[i][0], lines[i][1], lines[j][2], lines[j][3], freespace_gray)) {
	line(freespace, Point(lines[i][0], lines[i][1]), Point(lines[j][2], lines[j][3]), Scalar(255, 0, 0));
      }
      if (visible(lines[i][2], lines[i][3], lines[j][2], lines[j][3], freespace_gray)) {
	line(freespace, Point(lines[i][2], lines[i][3]), Point(lines[j][2], lines[j][3]), Scalar(255, 0, 0));
      }
    }
  }
  
  std::vector<float> angles(lines.size());
  float avgAngle = 0;
  for(unsigned int i=0; i<lines.size(); ++i) {
    float x = lines[i][3] - lines[i][1];
    float y = lines[i][2] - lines[i][0];
    angles[i] = abs(x) < 1e-6 ? CV_PI/2 : atan(y/x);

    //printf("%d: %f\n", i, angles[i]);
    
    avgAngle += angles[i];
  }

  avgAngle /= angles.size();

  float mainAngle = 0, perpAngle = 0;
  int numMain = 0, numPerp = 0;

  for(unsigned int i=0; i<angles.size(); ++i) {
    if (angles[i] > avgAngle) {
      mainAngle += angles[i];
      numMain++;
    } else {
      perpAngle += angles[i];
      numPerp++;
    }
  }

  if (numMain > 0) {
    mainAngle /= numMain;
    if (numPerp == 0) {
      perpAngle = mainAngle - CV_PI/2;
    }
  }
  if (numPerp > 0) {
    perpAngle /= numPerp;
    if (numMain == 0) {
      mainAngle = perpAngle + CV_PI/2;
    }
  }

  printf("Main angle: %f\nPerp angle: %f\n", mainAngle, perpAngle);
  //std::cout << freespace_gray << std::endl << freespace_gray.rows << std::endl;
  

  imshow("Hough Lines", walls);
  imshow("Anchor Points", freespace);
  waitKey(0);
}

// 228, 141 --> 180, 95; 210, 34 | 229, 10; 263, 44

bool visible(int xstart, int ystart, int xend, int yend, cv::Mat &freespace) {
  bool vis = true;
  bool vert = false;
  if (abs(yend - ystart) > abs(xend - xstart)) {
    vert = true;
    swap(xstart, ystart);
    swap(xend, yend);
  } else {
    return false;
  }

  if (xstart > xend) {
    swap(xstart, xend);
    swap(ystart, yend);
  }

  for(int i=xstart; i<=xend; ++i) {
    int j = ystart + (i - xstart) * (yend - ystart) / (xend - xstart);
    
    if (vert && freespace.at<uchar>(i, j) != 0 || !vert && freespace.at<uchar>(j, i) != 0) {
      vis = false;
      break;
    }
  }

  return vis;
}

void swap(int &one, int &two) {
  one = one ^ two;
  two = one ^ two;
  one = one ^ two;
}
