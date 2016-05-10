#ifndef SEGMENT_H
#define SEGMENT_H

#include <map>
#include <string>
#include <vector>

#define END_HEADER "end_header"
#define MASK_WIDTH 300
#define WALL_THRESH 0.25
#define SUBSAMPLE_STEP 3
#define VISIBILITY_BUFFER 2
#define MERGE_THRESH 0.6
#define NUM_CLUSTERS 50
#define KMEDOIDS_LIMIT 20

#define DEBUG 1

class Segment {
 public:
  std::vector< std::pair<int, int> > wallIndices, freeIndices; // subsampled indices of walls/free space
  std::vector< std::vector<int> > density;
  std::vector< std::vector<float> > freeSpaceProb;
  std::vector< std::vector<bool> > walls, freeSpace;
  std::vector< std::vector<float> > visibilityVectors;
  std::vector<float> vx, vy, vz;
  unsigned int vertices, faces, edges;
  unsigned int width, height; // mask width, height
  float mainAngle, perpAngle;
  
  Segment(char* filename, unsigned int width = MASK_WIDTH);
  
  void computeDensity();
  void findWalls(float threshold = WALL_THRESH);
  void findManhattanDirections();
  
  void densityMap(std::vector< std::vector<int> > &map, std::string outputName);
  void binaryMap(std::vector< std::vector<bool> > &map, std::string outputName);
  void clusterMap(std::map< int, std::vector<int> > &clusters, std::string outputName);
  
  void coord2index(float x, float y, int &xindex, int &yindex);
  void index2coord(int xindex, int yindex, float &x, float &y);
  
  void subsample(int stepsize = SUBSAMPLE_STEP);
  
  void dilate(std::vector< std::vector<bool> > &mask);
  void erode(std::vector< std::vector<bool> > &mask);
  void open(std::vector< std::vector<bool> > &mask);
  void close(std::vector< std::vector<bool> > &mask);
  void setKernel(std::vector< std::vector<bool> > &kernel);

  void computeFreeSpaceVisibility();
  void testVisibility(int testIndex, std::string output);

  void clustering(int clusters = NUM_CLUSTERS);
  
 private:
  char * filename;
  float xmax, xmin, ymax, ymin, zmax, zmin;
  int maxDensity, kernelSum;
  std::vector< std::vector<bool> > kernel;
  
  void split(const std::string &s, std::vector<std::string> &elems);
  void readFile();
  bool visible(int xstart, int ystart, int xend, int yend, int buffer=VISIBILITY_BUFFER);
  void swap(int &one, int &two);
  void normalize(std::vector<float> &v);
  float distance(std::vector<float> &one, std::vector<float> &two);
  void computeVisibility(int fx, int fy, std::vector<float> &out);
  void recenter(std::map<int, std::vector<int> > &clusterMembers, std::vector<int> &indices);
  void assignClusters(std::map<int, std::vector<int> > &clusterMembers, std::vector<int> &indices);
  bool merge(std::map<int, std::vector<int> > &clusterMembers, std::vector<int> &indices, int &clusters);

};

#endif
