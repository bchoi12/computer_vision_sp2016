
#include <string>
#include <vector>
#include <iostream>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "robust_pca.h"

using Eigen::MatrixXd;

int main(int argc, char** argv) {

  std::string name = "";
  if (argc > 1) {
    name = "_" + std::string(argv[1]);
  }
  
  cv::Mat freespace_img = cv::imread("freespace" + name + "_rotated.png", CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat freespaceProb_img = cv::imread("freespaceProb" + name + "_rotated.png", CV_LOAD_IMAGE_GRAYSCALE);

  MatrixXd fsp = MatrixXd::Zero(freespaceProb_img.rows, freespaceProb_img.cols);
  MatrixXd fspRowRank = MatrixXd::Zero(freespaceProb_img.rows, freespaceProb_img.cols);
  MatrixXd fspSparse = MatrixXd::Zero(freespaceProb_img.rows, freespaceProb_img.cols);

  for(unsigned int i=0; i<freespaceProb_img.rows; ++i) {
    for(unsigned int j=0; j<freespaceProb_img.cols; ++j) {
      fsp(i, j) = (double) freespaceProb_img.at<unsigned char>(i, j);
    }
  }

  sp::ml::robust_pca(fsp, fspRowRank, fspSparse);
  
  cv::Mat freespaceProb(freespaceProb_img.rows, freespaceProb_img.cols, CV_8U);

  for(unsigned int i=0; i<freespaceProb.rows; ++i) {
    for(unsigned int j=0; j<freespaceProb.cols; ++j) {
      freespaceProb.at<unsigned char>(i, j) = std::min(255, std::max(0, (int) fsp(i, j)));
    }
  }
  
  cv::imwrite("freespaceProb" + name + "_clean.png", freespaceProb);
}
