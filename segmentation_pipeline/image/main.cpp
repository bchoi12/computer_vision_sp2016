
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#define DEFAULT_WIDTH 300
#define END_HEADER "end_header"
#define THRESHOLD 0.25

void split(const std::string &s, std::vector<std::string> &elems);
void writePPM(cv::Mat &map, std::string outputName, int mapMax = 1);

int main(int argc, char** argv) {

  if (argc < 2) {
    printf("Usage: rotate fname [width]\n\tfname: input ply in ascii format\n\twidth: width of initial image\n");
    return 1;
  }

  char* fname = argv[1];
  int width = DEFAULT_WIDTH;
  
  if (argc > 2) {
    width = atoi(argv[2]);
  }

  std::string name = "";
  
  if (argc > 3) {
    name = std::string(argv[3]) + "_";
  }
  
  int vertices = 0, faces = 0, edges = 0;
  printf("Reading from file %s\n", fname);
  std::ifstream ifs(fname);

  std::vector<float> vx, vy, vz;
  float xmax, ymax, zmax, xmin, ymin, zmin;
  if (ifs.is_open()) {
    std::string line;
    std::vector<std::string> tokens;
    do {
      std::getline(ifs, line);
      split(line, tokens);

      if (tokens[0] == "element") {
	if (tokens[1] == "vertex") {
	  vertices = atoi(tokens[2].c_str());
	} else if (tokens[1] == "face") {
	  faces = atoi(tokens[2].c_str());
	} else if (tokens[1] == "edge") {
	  edges = atoi(tokens[2].c_str());
	}
      }
    }
    while(line != END_HEADER);

    printf("Found %i vertices\n", vertices);

    vx.resize(vertices);
    vy.resize(vertices);
    vz.resize(vertices);
    for(unsigned int i=0; i<vertices; ++i) {
      std::getline(ifs, line);
      split(line, tokens);
      vx[i] = atof(tokens[0].c_str());
      vy[i] = atof(tokens[1].c_str());
      vz[i] = atof(tokens[2].c_str());

      if (i == 0) {
	xmax = vx[i]; xmin = vx[i];
	ymax = vy[i]; ymin = vy[i];
	zmax = vz[i]; zmin = vz[i];
      } else {
	xmax = std::max(vx[i], xmax); xmin = std::min(vx[i], xmin);
	ymax = std::max(vy[i], ymax); ymin = std::min(vy[i], ymin);
	zmax = std::max(vz[i], zmax); zmin = std::min(vz[i], zmin);
      }
    }
  }
  ifs.close();

  int height = width * (ymax - ymin) / (xmax - xmin);

  printf("Width: %i\nHeight: %i\n", width, height);
  printf("xmin: %f\txmax: %f\nymin: %f\tymax: %f\n", xmin, xmax, ymin, ymax);

  int a = 0, b = 0;
  
  int maxDensity = 0;
  cv::Mat density = cv::Mat_<int>(height, width);

  density *= 0;
  
  for(unsigned int i=0; i<vx.size(); ++i) {
    int xindex, yindex;
    xindex = std::min((int) (width * (vx[i] - xmin) / (xmax - xmin)), (int) width-1);
    yindex = std::min((int) (height * (vy[i] - ymin) / (ymax - ymin)), (int) height-1);

    int curDensity = density.at<int>(yindex, xindex)++;
    maxDensity = std::max(curDensity, maxDensity);
  }

  printf("Max density: %i\n", maxDensity);
  
  writePPM(density, name + "density", maxDensity);

  cv::Mat walls = cv::Mat_<bool>(height, width);
  cv::Mat freespace = cv::Mat_<bool>(height, width);
  cv::Mat freespaceProb = cv::Mat_<int>(height, width);

  for(unsigned int i=0; i<walls.rows; ++i) {
    for(unsigned int j=0; j<walls.cols; ++j) {
      walls.at<bool>(i, j) = density.at<int>(i, j) > THRESHOLD * maxDensity ? 1 : 0;
      freespace.at<bool>(i, j) = density.at<int>(i, j) == 0 ? false : true; //!walls.at<bool>(i, j);
      freespaceProb.at<int>(i, j) = density.at<int>(i, j) == 0 ? 0 : walls.at<bool>(i, j) ? 0 : THRESHOLD*maxDensity - density.at<int>(i, j);
    }
  }

  writePPM(walls, name + "walls");
  writePPM(freespace, name + "freespace");
  writePPM(freespaceProb, name + "freespaceProb", THRESHOLD*maxDensity);
  
  printf("Wrote images!\n");
}

void split(const std::string &s, std::vector<std::string> &elems) {
  elems.clear();
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ' ')) {
    elems.push_back(item);
  }
}

void writePPM(cv::Mat& map, std::string outputName, int mapMax) {
  const unsigned int mapHeight = map.rows; const unsigned int mapWidth = map.cols;

  if (mapWidth == 0 || mapHeight == 0) {
    return;
  }
  
  outputName += ".ppm";
  
  FILE *fp = fopen(outputName.c_str(), "wb");
  fprintf(fp, "P6\n%d %d\n255\n", mapWidth, mapHeight);
  for(unsigned int i=0; i<mapHeight; ++i) {
    for(unsigned int j=0; j<mapWidth; ++j) {
      static unsigned char color[3];
      color[0] = (mapMax == 1 ? 255*map.at<bool>(i, j) : 255*map.at<int>(i, j) / mapMax);
      color[1] = color[0];
      color[2] = color[0];
      fwrite(color, 1, 3, fp);
    }
  }

  fclose(fp);
}
