

#include "segment.h"
#include <stdlib.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>

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
}
  
void Segment::densityMap(std::vector< std::vector<int> > &map, std::string outputName) {
  const unsigned int mapHeight = map.size(); const unsigned int mapWidth = map[0].size();

  if (mapWidth == 0 || mapHeight == 0) {
    return;
  }

  int mapMax = map[0][0];
  for(unsigned int i=0; i<mapWidth; ++i) {
    for(unsigned int j=0; j<mapHeight; ++j) {
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
  clusters = std::max(clusters, (int) freeIndices.size());

  // pick random vertices to be initial cluster centers
  std::vector<int> indices(freeIndices.size());
  for(unsigned int i=0; i<freeIndices.size(); ++i) {
    indices[i] = i;
  }
  std::random_shuffle(indices.begin(), indices.end());

  // vector of each cluster center's mask coordinates
  std::vector<std::pair<int, int> > centers(clusters);
  for(unsigned int i=0; i<clusters; ++i) {
    centers[i] = freeIndices[indices[i]]; 
  }

  // map of cluster center to vector of cluster's members 
  std::map<int, std::vector<int> > clusterMembers;
  for(unsigned int i=0; i<clusters; ++i) {
    std::vector<int> v;
    v.push_back(indices[i]);
    clusterMembers[i] = v;
  }

  // vector of cluster center visibility vectors
  std::vector< std::vector<float> > centerVisibility(clusters);
  for(unsigned int i=0; i<clusters; ++i) {
    centerVisibility[i] = visibilityVectors[indices[i]];
  }

  // merge()
  
  bool merged;
  int rounds = 1;
  do {

    // compute center coords and visibility vectors
    std::map<int, std::vector<int> >::iterator it;
    for(it=clusterMembers.begin(); it!=clusterMembers.end(); ++it) {
      if (it->second.size() == 0) {
	printf("ERROR: cluster has no members and was not erased\n");
	continue;
      }
      
      int x = 0, y = 0;
      for(unsigned int i=0; i<it->second.size(); ++i) {
	x += freeIndices[it->second[i]].first;
	y += freeIndices[it->second[i]].second;
      }
      x /= it->second.size();
      y /= it->second.size();

      std::pair<int, int> cp(x, y);
      centers[it->first] = cp;

      it->second.clear(); // clear members
    }

    // compute visibility vectors

    // find best center for each coord
    for(unsigned int j=0; j<freeIndices.size(); ++j) {
      float maxScore = -1;
      int bestCenter = -1; 
      for(unsigned int k=0; k<centerVisibility.size(); ++k) {
	float curScore = score(visibilityVectors[j], centerVisibility[k]);
	if (curScore > maxScore) {
	  maxScore = curScore;
	  bestCenter = k;
	}
      }
      clusterMembers[bestCenter].push_back(j);
    }

    // erase clusters with no members

    // merge all 
    //merged = merge();
    
    rounds++;
  }
  while(merged && rounds < KMEDOIDS_LIMIT);
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

// returns score from 0 to 1 (assuming vectors are normalized)
float Segment::score(std::vector<float> &one, std::vector<float> &two) {
  if (one.size() != two.size()) {
    return -1;
  }

  // assuming both vectors are normalized
  float score = 0;
  for(unsigned int i=0; i<one.size(); ++i) {
    if (one[i] != 0 && two[i] != 0) {
      score += one[i] + two[i];
    }
  }
  return score/2;
}
