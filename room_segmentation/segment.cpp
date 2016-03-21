#include "segment.h"
#include <stdlib.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <math.h>
#include <sstream>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

Segment::Segment(char* filename, unsigned int width) {
  vertices = 0; faces = 0; edges = 0;
  
  std::vector<bool> top, mid, bot;
  top.push_back(0); top.push_back(1); top.push_back(0);
  mid.push_back(1); mid.push_back(0); mid.push_back(1);
  bot.push_back(0); bot.push_back(1); bot.push_back(0);
 
  kernel.push_back(top); kernel.push_back(mid); kernel.push_back(bot);
  kernelSum = 4;
  
  this->filename = filename;
  readFile();

  this->width = width;
  this->height = width * (ymax - ymin) / (xmax - xmin);

  density.resize(height);
  for(unsigned int i=0; i<density.size(); ++i) {
    density[i].resize(width);
  }
  maxDensity = 0;

  if (DEBUG) {
    printf("Mask width: %d; mask height: %d\n", width, height);
  }
}

void Segment::computeDensity() {
  if (DEBUG) {
    printf("Computing density given %d pixels\n", (int) vx.size());
  }

  maxDensity = 0;
  for(unsigned int i=0; i<vx.size(); ++i) {
    int xindex, yindex;
    coord2index(vx[i], vy[i], xindex, yindex);

    density[yindex][xindex]++;
    maxDensity = std::max(density[yindex][xindex], maxDensity);
  }

  if (DEBUG) {
    printf("Max density: %d pixels\n", maxDensity);
  }

  densityMap(density, "density_map");
}

void Segment::findWalls(float threshold) {
  walls.resize(density.size());
  freeSpace.resize(density.size());
  for(unsigned int i=0; i<walls.size(); ++i) {
    walls[i].resize(density[i].size());
    freeSpace[i].resize(density[i].size());
  }

  for(unsigned int i=0; i<walls.size(); ++i) {
    for(unsigned int j=0; j<walls[i].size(); ++j) {
      walls[i][j] = density[i][j] > threshold * maxDensity;
      freeSpace[i][j] = density[i][j] == 0 ? false : !walls[i][j];
    }
  }

  binaryMap(walls, "wall_map");
  binaryMap(freeSpace, "freespace_map");
}

void Segment::findManhattanDirections() {
  using namespace cv;

  Mat src, src_gray;
  src = imread("wall_map.ppm", CV_LOAD_IMAGE_COLOR);
  src.convertTo(src, CV_8U);
  cvtColor(src, src_gray, CV_BGR2GRAY);

  std::vector<Vec4i> lines;
  HoughLinesP(src_gray, lines, 1, CV_PI/180, 40, 30, 3); // output, lines_output, resolution, resolution of theta, min number of intersections for line, min line length, max line gap
  for(unsigned int i=0; i<lines.size(); ++i) {
    line(src, Point(lines[i][0], lines[i][1]),
	 Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255));
  }

  if (lines.size() == 0) {
    printf("Could not find any line segments! May need to tune hough lines parameters...\n");
    return;
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

  mainAngle = 0, perpAngle = 0;
  int numMain = 0, numPerp = 0;

  //printf("Avg angle: %f\n", avgAngle);

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

  /*
  namedWindow("Lines", 1);
  imshow("Lines", src);
  waitKey(0);
  */
  
}
  
void Segment::densityMap(std::vector< std::vector<int> > &map, std::string outputName) {
  const unsigned int mapHeight = map.size(); const unsigned int mapWidth = map[0].size();

  if (mapWidth == 0 || mapHeight == 0) {
    return;
  }

  int mapMax = map[0][0];
  for(unsigned int i=0; i<mapHeight; ++i) {
    for(unsigned int j=0; j<mapWidth; ++j) {
      mapMax = std::max(mapMax, map[i][j]);
    }
  }

  outputName += ".ppm";
  
  FILE *fp = fopen(outputName.c_str(), "wb");
  fprintf(fp, "P6\n%d %d\n255\n", mapWidth, mapHeight);
  for(unsigned int i=0; i<mapHeight; ++i) {
    for(unsigned int j=0; j<mapWidth; ++j) {
      static unsigned char color[3];
      color[0] = 255 * map[i][j] / mapMax;
      color[1] = color[0];
      color[2] = color[0];
      fwrite(color, 1, 3, fp);
    }
  }

  fclose(fp);

  if (DEBUG) {
    printf("Wrote density map %s\n", outputName.c_str());
  }
}
void Segment::binaryMap(std::vector< std::vector<bool> > &map, std::string outputName) {
  const unsigned int mapHeight = map.size(); const unsigned int mapWidth = map[0].size();

  outputName += ".ppm";
  
  FILE *fp = fopen(outputName.c_str(), "wb");
  fprintf(fp, "P6\n%d %d\n255\n", mapWidth, mapHeight);
  for(unsigned int i=0; i<mapHeight; ++i) {
    for(unsigned int j=0; j<mapWidth; ++j) {
      static unsigned char color[3];
      color[0] = map[i][j] ? 255 : 0;
      color[1] = color[0];
      color[2] = color[0];
      fwrite(color, 1, 3, fp);
    }
  }

  fclose(fp);

  if (DEBUG) {
    printf("Wrote binary map %s\n", outputName.c_str());
  }
}

void Segment::clusterMap(std::map< int, std::vector<int> > &clusters, std::string outputName) {

  outputName += ".ppm";

  std::vector< std::vector<int> > colors;

  colors.resize(height);
  for(unsigned int i=0; i<colors.size(); ++i) {
    colors[i].resize(width);
  }

  std::map< int, std::vector<int> >::iterator it;
  for(it=clusters.begin(); it!=clusters.end(); ++it) {
    std::vector<int> members = it->second;
    for(unsigned int i=0; i<members.size(); ++i) {
      std::pair<int, int> coords = freeIndices[members[i]];
      colors[coords.first][coords.second] = it->first+1;
    }
  }

  FILE *fp = fopen(outputName.c_str(), "wb");
  fprintf(fp, "P6\n%d %d\n255\n", width, height);
  for(unsigned int i=0; i<height; ++i) {
    for(unsigned int j=0; j<width; ++j) {
      static unsigned char color[3];
      color[0] = colors[i][j] * 317421 % 255;
      color[1] = colors[i][j] * 941827 % 255;
      color[2] = colors[i][j] * 893053 % 255;
      fwrite(color, 1, 3, fp);
    }
  }

  fclose(fp);

  if (DEBUG) {
    printf("Wrote cluster map %s\n", outputName.c_str());
  }
}
  
void Segment::coord2index(float x, float y, int &xindex, int &yindex) {
  xindex = std::min((int) ((x - xmin) / (xmax - xmin) * width), (int) width-1);
  yindex = std::min((int) ((y - ymin) / (ymax - ymin) * height), (int) height-1);
}

void Segment::index2coord(int xindex, int yindex, float &x, float &y) {
  x = xmin + xindex * (xmax - xmin) / width;
  y = ymin + yindex * (ymax - ymin) / height;
}
  
void Segment::subsample(int stepsize) {
  freeIndices.clear();
  wallIndices.clear();
  /*
  for(unsigned int i=0; i<vx.size(); i+=stepsize) {
    int xindex, yindex;
    coord2index(vx[i], vy[i], xindex, yindex);

    std::pair<int, int> indices(xindex, yindex);
    if (walls[xindex][yindex]) {
      wallIndices.push_back(indices);
    } else if (freeSpace[xindex][yindex]) {
      //freeIndices.push_back(indices);
    }
  }
  */

  /*
  for(unsigned int i=0; i<walls.size(); ++i) {
    for(unsigned int j=0; j<walls[i].size(); ++j) {
      if (walls[i][j]) {
	std::pair<int, int> indices(i, j);
	wallIndices.push_back(indices);
      }
    }
  }
  */

  for(unsigned int i=0; i<walls.size(); i+=stepsize) {
    for(unsigned int j=0; j<walls[i].size(); j+=stepsize) {
      std::pair<int, int> indices(i, j);
      if (walls[i][j]) {
	wallIndices.push_back(indices);
      } else if (freeSpace[i][j]) {

	// filter points close to wall?
	
	freeIndices.push_back(indices);
      }
    }
  }

  if (DEBUG) {
    printf("Subsampled %d free space points and %d wall points\n", (int) freeIndices.size(), (int) wallIndices.size());
  }
}
  
void Segment::dilate(std::vector< std::vector<bool> > &mask) {
  
  std::vector< std::vector<bool> > copy;
  copy.resize(mask.size());
  for(unsigned int i=0; i<mask.size(); ++i) {
    copy[i].resize(mask[i].size());
    for(unsigned int j=0; j<mask[i].size(); ++j) {
      copy[i][j] = mask[i][j];
      if (mask[i][j]) {
	continue;
      }
      int count = 0;
      for(unsigned int x=0; x<kernel.size(); ++x) {
	for(unsigned int y=0; y<kernel[x].size(); ++y) {
	  if (!kernel[x][y]) {
	    continue;
	  }
	  
	  int xindex = std::max(0, std::min((int) mask.size()-1, (int) (i+x-kernel.size()/2)));
	  int yindex = std::max(0, std::min((int) mask[i].size()-1, (int) (j+y-kernel.size()/2)));
	  if (mask[xindex][yindex]) {
	    count++;
	  }
	}
      }
      copy[i][j] = count >= kernelSum/2;
    }
  }
  mask = copy;
}

void Segment::erode(std::vector< std::vector<bool> > &mask) {
  
  std::vector< std::vector<bool> > copy;
  copy.resize(mask.size());
  for(unsigned int i=0; i<mask.size(); ++i) {
    copy[i].resize(mask[i].size());
    for(unsigned int j=0; j<mask[i].size(); ++j) {
      copy[i][j] = mask[i][j];
      if (!mask[i][j]) {
	continue;
      }
      int count = 0;
      for(unsigned int x=0; x<kernel.size(); ++x) {
	for(unsigned int y=0; y<kernel[x].size(); ++y) {
	  if (!kernel[x][y]) {
	    continue;
	  }
	  
	  int xindex = std::max(0, std::min((int) mask.size()-1, (int) (i+x-kernel.size()/2)));
	  int yindex = std::max(0, std::min((int) mask[i].size()-1, (int) (j+y-kernel.size()/2)));
	  if (!mask[xindex][yindex]) {
	    count++;
	  }
	}
      }
      copy[i][j] = !(count >= kernelSum/2);
    }
  }
  mask = copy;
}

void Segment::open(std::vector< std::vector<bool> > &mask) {
  erode(mask);
  dilate(mask);
}

void Segment::close(std::vector< std::vector<bool> > &mask) {
  dilate(mask);
  erode(mask);
}

void Segment::setKernel(std::vector< std::vector<bool> > &kernel) {
  this->kernel = kernel;
  kernelSum = 0;
  for(unsigned int i=0; i<kernel.size(); i++) {
    for(unsigned int j=0; j<kernel[i].size(); j++) {
      kernelSum += kernel[i][j] ? 1 : 0;
    }
  }
}

void Segment::computeFreeSpaceVisibility() {
  if (DEBUG) {
    printf("Computing %d-dimension visibility vectors for %d free space points...\n", (int) wallIndices.size(), (int) freeIndices.size());
  }
  
  visibilityVectors.clear();
  visibilityVectors.resize(freeIndices.size());
  for(unsigned int i=0; i<freeIndices.size(); ++i) {
    int fx = freeIndices[i].first;
    int fy = freeIndices[i].second;

    computeVisibility(fx, fy, visibilityVectors[i]);
  }

  if (DEBUG) {
    printf("Visibility computations finished\n");
  }
}

void Segment::computeVisibility(int fx, int fy, std::vector<float> &out) {
  out.resize(wallIndices.size());
  for(unsigned int i=0; i<wallIndices.size(); ++i) {
    int wx = wallIndices[i].first;
    int wy = wallIndices[i].second;
    out[i] = visible(fx, fy, wx, wy) ? 1 : 0;
  }
  normalize(out);
}

// do not use, test method
void Segment::testVisibility(int testIndex, std::string output) {
  int fx = freeIndices[testIndex].first;
  int fy = freeIndices[testIndex].second;
  printf("Testing coord %d, %d\n", fx, fy);
  
  std::vector< std::vector<bool> > mask;
  mask.resize(height);
  for(unsigned int i=0; i<mask.size(); ++i) {
    mask[i].resize(width);
  }
  
  for(unsigned int i=0; i<wallIndices.size(); ++i) {
    int wx = wallIndices[i].first;
    int wy = wallIndices[i].second;
    //printf("%d, %d -> %d, %d = %d\n", fx, fy, wx, wy, visible(fx, fy, wx, wy));
    mask[wx][wy]  = visible(fx, fy, wx, wy);
  }

  // CAREFUL
  mask[fx][fy] = true;
  mask[fx+1][fy] = true;
  mask[fx-1][fy] = true;
  mask[fx][fy+1] = true;
  mask[fx][fy-1] = true;

  binaryMap(mask, output);
}

void Segment::clustering(int clusters) {

  if (DEBUG) {
    printf("Beginning clustering...\n");
  }
  
  clusters = std::min(clusters, (int) freeIndices.size());

  // pick random vertices to be initial cluster centers
  std::vector<int> indices(freeIndices.size());
  for(unsigned int i=0; i<freeIndices.size(); ++i) {
    indices[i] = i;
  }
  std::srand(unsigned(time(NULL)));
  std::random_shuffle(indices.begin(), indices.end());
  indices.resize(clusters);

  // map of cluster center to vector of cluster's members 
  std::map<int, std::vector<int> > clusterMembers;
  for(unsigned int i=0; i<clusters; ++i) {
    std::vector<int> v;
    clusterMembers[i] = v;
  }

  assignClusters(clusterMembers, indices);

  int rounds = 0;
  bool merged = true;
  do {
    printf("Recentering and merging round %d, %d clusters left\n", rounds+1, clusters);
    recenter(clusterMembers, indices);
    assignClusters(clusterMembers, indices);
    merged = merge(clusterMembers, indices, clusters);
    rounds++;
  }
  while(merged && rounds < KMEDOIDS_LIMIT && clusters > 1);

  if (DEBUG) {
    printf("Num clusters: %d\n", clusters);
    int sum = 0;
    std::map<int, std::vector<int> >::iterator it;
    for(it=clusterMembers.begin(); it!=clusterMembers.end(); ++it) {
      if (it->second.size() != 0) {
	printf("Cluster %d size: %d\n", it->first, (int) it->second.size());
	sum += it->second.size();
      }
    }
    printf("Total cluster members: %d, free space points: %d\n", sum, (int) freeIndices.size());
    printf("Merged %d times\n", rounds);
    clusterMap(clusterMembers, "cluster_map");
  }
}

// find the best center within a cluster
void Segment::recenter(std::map<int, std::vector<int> > &clusterMembers, std::vector<int> &indices) {

  printf("Recentering...\n");
  
  std::map<int, std::vector<int> >::iterator it;
  for(it=clusterMembers.begin(); it!=clusterMembers.end(); ++it) {
    std::vector<int> members = it->second;
    float bestScore = members.size();
    int bestIndex = 0;

    for(unsigned int i=0; i<members.size(); ++i) {
      float score = 0;
      for(unsigned int j=0; j<members.size(); ++j) {
	if (i == j) {
	  continue;
	}
	score += distance(visibilityVectors[members[i]], visibilityVectors[members[j]]);
      }
      if (score < bestScore) {
	bestScore = score;
	bestIndex = members[i];
      }
    }

    indices[it->first] = bestIndex;
  }
}

void Segment::assignClusters(std::map<int, std::vector<int> > &clusterMembers, std::vector<int> &indices) {

  if (DEBUG) {
    printf("Reassigning points to clusters\n");
  }
  
  std::map<int, std::vector<int> >::iterator it;
  for(it=clusterMembers.begin(); it!=clusterMembers.end(); ++it) {
    it->second.clear();
  }
  
  // add free space points to clusters
  for(unsigned int i=0; i<freeIndices.size(); ++i) {
    float bestScore = 1;
    int bestCenter = 0;
    for(unsigned int j=0; j<indices.size(); ++j) {
      if (indices[j] == -1) {
	continue;
      }
      
      float score = distance(visibilityVectors[i], visibilityVectors[indices[j]]);
      if (score < bestScore) {
	bestScore = score;
	bestCenter = j;
      }
    }
    clusterMembers[bestCenter].push_back(i);
  }
}

// delete clusters with no members, merge clusters that are close
bool Segment::merge(std::map<int, std::vector<int> > &clusterMembers, std::vector<int> &indices, int &clusters) {

  if (DEBUG) {
    printf("Merging clusters...\n");
  }
  
  int origClusters = clusters;
  
  std::vector<int> mergeTo(indices.size());
  std::vector<int> bestMerge(indices.size());
  std::vector<float> bestScore(indices.size());

  for(unsigned int i=0; i<indices.size(); ++i) {
    mergeTo[i] = i;
    bestMerge[i] = -1;
    bestScore[i] = 1;
  }
  
  std::map<int, std::vector<int> >::iterator i, j;
  for(i=clusterMembers.begin(); i!=clusterMembers.end(); ++i) {
    if (i->second.size() == 0) {
      continue;
    }

    int curMerge = i->first;
    float minDist = 1;
    
    for(j=clusterMembers.begin(); j!=clusterMembers.end(); ++j) {
      if (i==j || j->second.size() == 0) {
	continue;
      }

      float curDist = distance(visibilityVectors[indices[i->first]], visibilityVectors[indices[j->first]]);
      if (curDist < minDist) {
	minDist = curDist;
	curMerge = j->first;
	//i->second.insert(i->second.end(), j->second.begin(), j->second.end());
	//j->second.clear();
      }
    }
    bestMerge[i->first] = curMerge;
    bestScore[i->first] = minDist;
  }

  for(unsigned int k=0; k<bestMerge.size(); ++k) {

    // prevents merging into self
    if (mergeTo[bestMerge[k]] == k) {
      continue;
    }

    // 2 clusters that want to merge into each other
    if (bestMerge[k] == bestMerge[bestMerge[k]]) {
      if (k < bestMerge[k]) {
	continue;
      }
      clusterMembers[mergeTo[bestMerge[k]]].insert(clusterMembers[mergeTo[bestMerge[k]]].end(), clusterMembers[k].begin(), clusterMembers[k].end());
      clusterMembers[k].clear();
      
      mergeTo[k] = mergeTo[bestMerge[k]];
    } else if (bestScore[k] < MERGE_THRESH) {
      
      clusterMembers[mergeTo[bestMerge[k]]].insert(clusterMembers[mergeTo[bestMerge[k]]].end(), clusterMembers[k].begin(), clusterMembers[k].end());
      clusterMembers[k].clear();
      
      mergeTo[k] = mergeTo[bestMerge[k]];
    }
  }
  
  for(unsigned int k=0; k<indices.size(); ++k) {
    if (clusterMembers.count(k) > 0 && clusterMembers[k].size() == 0) {
      clusters -= clusterMembers.erase(k);
      indices[k] = -1;
    }
  }
  
  return origClusters != clusters;
}

void Segment::split(const std::string &s, std::vector<std::string> &elems) {
  elems.clear();
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ' ')) {
    elems.push_back(item);
  }
}

void Segment::readFile() {
  printf("Reading from file %s\n", filename);
  std::ifstream ifs(filename);
  
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

    if (DEBUG) {
      printf("Finished reading header, found %d vertices, %d faces, %d edges\n", vertices, faces, edges);
    }

    // reading vertices hard coded for now
    // expecting 3 floats (x, y, z) at beginning of each line
    
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
}

bool Segment::visible(int xstart, int ystart, int xend, int yend, int buffer) {
  bool vis = true;
  bool vert = false;
  if (abs(yend - ystart) > abs(xstart - xend)) {
    vert = true;
    swap(xstart, ystart);
    swap(xend, yend);
  }

  if (xstart > xend) {
    swap(xstart, xend);
    swap(ystart, yend);
  }

  for(int i=xstart+buffer; i<=xend-buffer; ++i) {
    int j = ystart + (i - xstart) * (yend-ystart) / (xend - xstart);
    
    if (vert && walls[j][i] || !vert && walls[i][j]) {
      vis = false;
      break;
    }
  }

  return vis;
}

void Segment::swap(int &one, int &two) {
  one = one ^ two;
  two = one ^ two;
  one = one ^ two;
}

void Segment::normalize(std::vector<float> &v) {
  float sum = 0;
  for(unsigned int i=0; i<v.size(); ++i) {
    sum += v[i];
  }

  if (sum == 0 || sum == 1) {
    return;
  }

  for(unsigned int i=0; i<v.size(); ++i) {
    v[i] = v[i] / sum;
  }
}

// returns normalized distance from 0 to 1 (assuming vectors are normalized)
float Segment::distance(std::vector<float> &one, std::vector<float> &two) {
  if (one.size() != two.size()) {
    return -1;
  }

  // assuming both vectors are normalized
  float score = 0;
  for(unsigned int i=0; i<one.size(); ++i) {
    if (one[i] == 0 && two[i] != 0 || one[i] != 0 && two[i] == 0) {
      score += one[i] + two[i];
    }
  }
  return score/2;
}
