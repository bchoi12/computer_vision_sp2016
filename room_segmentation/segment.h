#ifndef SEGMENT_H
#define SEGMENT_H

#include <string>
#include <vector>

#define END_HEADER "end_header"
#define MASK_WIDTH 200
#define WALL_THRESH 0.25

#define DEBUG 1

class Segment {
 public:
  std::vector< std::vector<int> > density;
  std::vector< std::vector<bool> > walls, freeSpace; 
  std::vector<float> vx, vy, vz;
  unsigned int vertices, faces, edges;
  unsigned int width, height; // mask width, height

  Segment(char* filename, unsigned int width = MASK_WIDTH);
  
  void computeDensity();
  void findWalls(float threshold = WALL_THRESH);
  
  void densityMap(std::vector< std::vector<int> > &map, std::string outputName);
  void binaryMap(std::vector< std::vector<bool> > &map, std::string outputName);
  
  void coord2index(float x, float y, int &xindex, int &yindex);
  void index2coord(int xindex, int yindex, float &x, float &y);
  
  void subsample(int sampleSize, std::vector<int> &indices);
  void subsample(float sampleSize, std::vector<int> &indices);
  
  void dilate(std::vector< std::vector<bool> > &mask);
  void erode(std::vector< std::vector<bool> > &mask);
  void open(std::vector< std::vector<bool> > &mask);
  void close(std::vector< std::vector<bool> > &mask);
  void setKernel(std::vector< std::vector<bool> > &kernel);
  
 private:
  char * filename;
  float xmax, xmin, ymax, ymin, zmax, zmin;
  int maxDensity, kernelSum;
  std::vector< std::vector<bool> > kernel;
  
  void split(const std::string &s, std::vector<std::string> &elems);
  void readFile();
};

#endif
