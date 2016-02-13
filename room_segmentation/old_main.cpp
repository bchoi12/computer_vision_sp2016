#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>

#include "segment.cpp"

#define DEBUG 1

#define MASK_SIZE 200
#define WALL_THRESH 0.25

void split(const std::string &s, std::vector<std::string> &elems);
void computeDensity(std::vector< std::vector<int> > &density, std::vector<float> &vx, std::vector<float> &vy, float xmin, float xmax, float ymin, float ymax);
void findWalls(std::vector< std::vector<int> > &density, std::vector< std::vector<bool> > &walls, std::vector< std::vector<bool> > &freeSpace, int maxDensity, float threshold = WALL_THRESH);
void densityMap(std::vector< std::vector<int> > &density, int maxDensity, std::string outputName);
void binaryMap(std::vector< std::vector<bool> > &density, std::string outputName);
void coord2index(float x, float y, float xmin, float xmax, float ymin, float ymax, unsigned int width, unsigned int height, int &xindex, int &yindex);
void index2coord(int xindex, int yindex, float xmin, float xmax, float ymin, float ymax, unsigned int width, unsigned int height, float &x, float &y);

void subsample(std::vector< std::vector<bool> > &mask, std::vector<int> &indices, int sampleSize);
void subsample(std::vector< std::vector<bool> > &mask, std::vector<int> &indices, float portion);

void dilate(std::vector< std::vector<bool> > &mask, std::vector< std::vector<bool> > &kernel);
void erode(std::vector< std::vector<bool> > &mask, std::vector< std::vector<bool> > &kernel);
void open(std::vector< std::vector<bool> > &mask, std::vector< std::vector<bool> > &kernel);
void close(std::vector< std::vector<bool> > &mask, std::vector< std::vector<bool> > &kernel);

int main(int argc, char** argv) {
  if (argc < 2) {
    printf("Usage: segment fname\n  fname: input ply in ascii format");
    return 1;
  }

  printf("Reading from file %s\n", argv[1]);
  std::ifstream ifs(argv[1]);
  
  if (ifs.is_open()) {
    unsigned int vertices = 0, faces = 0, edges = 0;

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

    float xmax, ymax, zmax, xmin, ymin, zmin;
    
    std::vector<float> vx, vy, vz;
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

    if (DEBUG) {
      printf("Finished reading %d vertices into memory\n", vertices);
      printf("xmax: %.4f\txmin: %.4f\nymax: %.4f\tymin: %.4f\nzmax: %.4f\tzmin: %.4f\n", xmax, xmin, ymax, ymin, zmax, zmin);
    }

    // hardcoded mask size for now
    const int maskSize = MASK_SIZE;

    std::vector< std::vector<int> > density;
    density.resize(maskSize);
    for(unsigned int i=0; i<density.size(); ++i) {
      density[i].resize(maskSize);
    }

    computeDensity(density, vx, vy, xmin, xmax, ymin, ymax);
  }

  ifs.close();
}

// split a string by whitespace
void split(const std::string &s, std::vector<std::string> &elems) {
  elems.clear();
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ' ')) {
    elems.push_back(item);
  }
}

void computeDensity(std::vector< std::vector<int> > &density, std::vector<float> &vx, std::vector<float> &vy, float xmin, float xmax, float ymin, float ymax) {

  int maxDensity = 0;
  for(unsigned int i=0; i<vx.size(); ++i) {
    int xindex, yindex;
    coord2index(vx[i], vy[i], xmin, xmax, ymin, ymax, density.size(), density[0].size(), xindex, yindex);

    density[yindex][xindex]++;
    maxDensity = std::max(density[yindex][xindex], maxDensity);
  }

  if (DEBUG) {
    printf("Max density: %d pixels\n", maxDensity);
  }

  densityMap(density, maxDensity, "density_map");
  if (DEBUG) {
    printf("Wrote density map image density_map.ppm\n");
  }

  std::vector< std::vector<bool> > walls, freeSpace;
  findWalls(density, walls, freeSpace,  maxDensity);
  binaryMap(walls, "wall_map");
  binaryMap(freeSpace, "free_map");
  if (DEBUG) {
    printf("Wrote wall map image wall_map.ppm\n");
    printf("Wrote free space map image free_map.ppm\n");
  }

  std::vector<bool> top, mid, bot;
  top.push_back(0); top.push_back(1); top.push_back(0);
  mid.push_back(1); mid.push_back(0); mid.push_back(1);
  bot.push_back(0); bot.push_back(1); bot.push_back(0);
  std::vector< std::vector<bool> > kernel;
  kernel.push_back(top); kernel.push_back(mid); kernel.push_back(bot);

  open(freeSpace, kernel);
  binaryMap(freeSpace, "open_free_map");
  
  close(walls, kernel);
  binaryMap(walls, "close_wall_map");
}

void findWalls(std::vector< std::vector<int> > &density, std::vector< std::vector<bool> > &walls, std::vector< std::vector<bool> > &freeSpace, int maxDensity, float threshold) {
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
  
  /*
  // refine free space
  const int rad = 1;
  for(unsigned int i=0; i<freeSpace.size(); ++i) {
    for(unsigned int j=0; j<freeSpace.size(); ++j) {
      if (!freeSpace[i][j]) {
	continue;
      }

      for(int x=-rad; x<=rad; ++x) {
	for(int y=-rad; y<=rad; ++y) {
	  int xindex = std::max(0, std::min((int) freeSpace.size()-1, (int) i+x));
	  int yindex = std::max(0, std::min((int) freeSpace.size()-1, (int) j+y));
	  if (walls[xindex][yindex]) {
	    freeSpace[i][j] = false;
	    continue;
	  }
	}
      }
    }
  }
  */
}

void densityMap(std::vector< std::vector<int> > &density, int maxDensity,  std::string outputName) {
  const unsigned int width = density.size(), height = density[0].size();

  FILE *fp = fopen((outputName + ".ppm").c_str(), "wb");
  fprintf(fp, "P6\n%d %d\n255\n", width, height);
  for(unsigned int j=0; j<height; ++j) {
    for(unsigned int i=0; i<width; ++i) {
      static unsigned char color[3];
      color[0] = 255 * density[j][i] / maxDensity;
      color[1] = color[0];
      color[2] = color[0];
      fwrite(color, 1, 3, fp);
    }
  }

  fclose(fp);
}

void binaryMap(std::vector< std::vector<bool> > &density, std::string outputName) {
  const unsigned int width = density.size(), height = density[0].size();

  FILE *fp = fopen((outputName + ".ppm").c_str(), "wb");
  fprintf(fp, "P6\n%d %d\n255\n", width, height);
  for(unsigned int j=0; j<height; ++j) {
    for(unsigned int i=0; i<width; ++i) {
      static unsigned char color[3];
      color[0] = density[j][i] ? 255 : 0;
      color[1] = color[0];
      color[2] = color[0];
      fwrite(color, 1, 3, fp);
    }
  }

  fclose(fp);
}

void coord2index(float x, float y, float xmin, float xmax, float ymin, float ymax, unsigned int width, unsigned int height, int &xindex, int &yindex) {
    xindex = std::min((int) ((x - xmin) / (xmax - xmin) * width), (int) width-1);
    yindex = std::min((int) ((y - ymin) / (ymax - ymin) * height), (int) height-1);
}

void index2coord(int xindex, int yindex, float xmin, float xmax, float ymin, float ymax, unsigned int width, unsigned int height, float &x, float &y) {
  x = xmin + xindex * (xmax - xmin) / width;
  y = ymin + yindex * (ymax - ymin) / height;
}

void dilate(std::vector< std::vector<bool> > &mask, std::vector< std::vector<bool> > &kernel) {

  int kernelSum = 0;
  for(unsigned int i=0; i<kernel.size(); ++i) {
    for(unsigned int j=0; j<kernel.size(); ++j) {
      if (kernel[i][j]) {
	kernelSum++;
      }
    }
  }
  
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

void erode(std::vector< std::vector<bool> > &mask, std::vector< std::vector<bool> > &kernel) {

  int kernelSum = 0;
  for(unsigned int i=0; i<kernel.size(); ++i) {
    for(unsigned int j=0; j<kernel.size(); ++j) {
      if (kernel[i][j]) {
	kernelSum++;
      }
    }
  }
  
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

void open(std::vector< std::vector<bool> > &mask, std::vector< std::vector<bool> > &kernel) {
  erode(mask, kernel);
  dilate(mask, kernel);
}

void close(std::vector< std::vector<bool> > &mask, std::vector< std::vector<bool> > &kernel) {
  dilate(mask, kernel);
  erode(mask, kernel);
}
