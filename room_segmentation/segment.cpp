

#include "segment.h"
#include <stdlib.h>
#include <fstream>
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
    density[i].resize(height);
  }
  maxDensity = 0;
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

  int mapMax;
  for(unsigned int i=0; i<mapWidth; ++i) {
    for(unsigned int j=0; j<mapHeight; ++j) {
      if (i == 0 && j == 0) {
	mapMax = map[i][j];
      } else {
	mapMax = std::max(mapMax, map[i][j]);
      }
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
  
void Segment::subsample(int sampleSize, std::vector<int> &indices) {
  // implement
}
void Segment::subsample(float sampleSize, std::vector<int> &indices) {
  // implement
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
