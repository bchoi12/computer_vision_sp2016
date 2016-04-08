
#include <vector>


#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

void rotate(cv::Mat& src, double angle, cv::Mat& dst, bool thresh = true);

int main(int argc, char** argv) {
  cv::Mat walls_img = cv::imread("../walls.ppm");
  cv::Mat freespace_img = cv::imread("../freespace.ppm");
  cv::Mat density_img = cv::imread("../density.ppm");
  cv::Mat walls_gray = cv::imread("../walls.ppm", CV_LOAD_IMAGE_GRAYSCALE);
  
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(walls_gray, lines, 1, CV_PI/180, 10, 5, 3);

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

  if (numMain >= numPerp) {
    mainAngle /= numMain;
    perpAngle = mainAngle - CV_PI/2;
  } else {
    perpAngle /= numPerp;
    mainAngle = perpAngle;
    perpAngle = mainAngle - CV_PI/2;
  }

  printf("Main angle: %f\nPerp angle: %f\n", mainAngle, perpAngle);
  
  cv::Mat rot_walls, rot_freespace, rot_density;
  rotate(walls_img, -mainAngle*180/CV_PI, rot_walls);
  rotate(freespace_img, -mainAngle*180/CV_PI, rot_freespace);
  rotate(density_img, -mainAngle*180/CV_PI, rot_density, false);
  
  cv::imwrite("../density_rotated.png", rot_density);
  cv::imwrite("../freespace_rotated.png", rot_freespace);
  cv::imwrite("../walls_rotated.png", rot_walls);
  
  printf("Rows: %d\nCols: %d\n", rot_freespace.rows, rot_freespace.cols);
}

void rotate(cv::Mat& src, double angle, cv::Mat& dst, bool thresh)
{
  int len = std::sqrt(src.cols*src.cols + src.rows*src.rows);

  dst = cv::Mat(cv::Size(len, len), src.type());

  cv::Mat t = (cv::Mat_<double>(2, 3) << 1, 0, (len-src.cols)/2, 0, 1, (len-src.rows)/2);
  cv::warpAffine(src, dst, t, dst.size());

  cv::Point2f pt(len/2., len/2.);
  cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

  cv::warpAffine(dst, dst, r, dst.size());

  if (thresh) {
    cv::cvtColor(dst, dst, CV_BGR2GRAY);
    cv::threshold(dst, dst, 100, 255, cv::THRESH_BINARY);
  }
}
