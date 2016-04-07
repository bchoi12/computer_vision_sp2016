#include <stdio.h>
#include <stdlib.h>

#include <iostream>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "MRF/mrf.h"
#include "MRF/GCoptimization.h"

bool visible(int xstart, int ystart, int xend, int yend, cv::Mat &freespace);
void swap(int &one, int &two);
void rotate(cv::Mat& src, double angle, cv::Mat& dst);

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

    //line(freespace, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0, 255, 0));
    //circle(freespace, Point(lines[i][0], lines[i][1]), 2, Scalar(0, 0, 255));
    //circle(freespace, Point(lines[i][2], lines[i][3]), 2, Scalar(0, 0, 255));
  }

  /*
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
  */
  
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

  Mat rot_walls, rot_freespace;
  rotate(walls, -mainAngle*180/CV_PI, rot_walls);
  rotate(freespace, -mainAngle*180/CV_PI, rot_freespace);

  printf("Main angle: %f\nPerp angle: %f\n", mainAngle, perpAngle);
  printf("Rows: %d\nCols: %d\n", rot_freespace.rows, rot_freespace.cols);

  for(int i=0; i<rot_freespace.rows; ++i) {
    for(int j=0; j<rot_freespace.cols; ++j) {
      rot_freespace.at<uchar>(i, j) = rot_freespace.at<uchar>(i, j) > 0 ? 1 : 0;
    }
  }

  int numLabels = 2;
  int rows = rot_freespace.cols, cols = rot_freespace.cols;
  MRF::CostVal D[rows*cols*numLabels];
  MRF::CostVal V[numLabels * numLabels];
  MRF::CostVal* ptr;

  printf("Rows: %d\nCols: %d\n", rows, cols);

  for(int j=0; j<cols; ++j) {
    for(int i=0; i<rows; ++i) {
      int pix = i*cols + j;
      for(int l=0; l<numLabels; ++l) {
	D[pix*numLabels + l] = (MRF::CostVal) (l == rot_freespace.at<uchar>(i, j) ? 0 : 30);
      }
    }
  }

  for(int i=0; i<numLabels; ++i) {
    for(int j=i; j<numLabels; ++j) {
      V[i*numLabels+j] = V[j*numLabels+i] = (i == j) ? 0 : (MRF::CostVal) 20; 
    }
  }

  DataCost *data = new DataCost(D);
  SmoothnessCost *smooth = new SmoothnessCost(V);
  EnergyFunction *energy = new EnergyFunction(data, smooth);

  MRF* mrf = new Expansion(rows, cols, numLabels, energy);
  mrf->initialize();
  mrf->clearAnswer();

  printf("Energy at the Start= %g (%g,%g)\n", (float)mrf->totalEnergy(),
	 (float)mrf->smoothnessEnergy(), (float)mrf->dataEnergy());

  float tot_t = 0, t;
  for (int iter=0; iter<6; iter++) {
    mrf->optimize(1, t);
    
    tot_t = tot_t + t ;
    printf("energy = %g (%f secs)\n", (float)mrf->totalEnergy(), tot_t);
  }

  /*
  for (int pix=0; pix<rows*cols; ++pix) {
    if (pix > 0 && pix % rows == 0) {
      std::cout << std::endl;
    }
    std::cout << mrf->getLabel(pix);
  }
  */
  
  FILE *fp = fopen("thing.ppm", "wb");
  fprintf(fp, "P6\n%d %d\n255\n", rows, cols);
  for(int pix=0; pix<rows*cols; ++pix) {
    static unsigned char color[3];
    color[0] = mrf->getLabel(pix) == 1 ? 255 : 0;
    color[1] = color[0];
    color[2] = color[0];
    fwrite(color, 1, 3, fp);
  }

  fclose(fp);

  /*
  uchar labels [rows*cols];
  memcpy(mrf->getAnswerPtr(), labels, rows*cols*sizeof(uchar));
  
  for(int j=0; j<cols; ++j) {
    for(int i=0; i<rows; ++i) {
      std::cout << ((int) labels[j*rows+i]);
    }
    std::cout << std::endl;
  }
  */
  
  delete mrf;
  
  imshow("Hough Lines", walls);
  imshow("Anchor Points", freespace_gray);
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

void rotate(cv::Mat& src, double angle, cv::Mat& dst)
{
  int len = std::sqrt(src.cols*src.cols + src.rows*src.rows);

  dst = cv::Mat(cv::Size(len, len), src.type());

  cv::Mat t = (cv::Mat_<double>(2, 3) << 1, 0, (len-src.cols)/2, 0, 1, (len-src.rows)/2);
  cv::warpAffine(src, dst, t, dst.size());

  cv::Point2f pt(len/2., len/2.);
  cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

  cv::warpAffine(dst, dst, r, dst.size());

  cv::cvtColor(dst, dst, CV_BGR2GRAY);
  cv::threshold(dst, dst, 100, 255, cv::THRESH_BINARY);
}
