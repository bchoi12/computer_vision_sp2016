#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#define DEBUG 1
#define END_HEADER "end_header"

void split(const std::string &s, std::vector<std::string> &elems);
void computeDensity(std::vector< std::vector<int> > &density, std::vector<float> &vx, std::vector<float> &vy, float &xmin, float &xmax, float &ymin, float &ymax);
void densityMap(std::vector< std::vector<int> > &density, int &maxDensity, std::string outputName);

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
    const int maskSize = 100;

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

void computeDensity(std::vector< std::vector<int> > &density, std::vector<float> &vx, std::vector<float> &vy, float &xmin, float &xmax, float &ymin, float &ymax) {

  int maxDensity = 0;
  for(unsigned int i=0; i<vx.size(); ++i) {
    int xindex = std::min((int) ((vx[i] - xmin) / (xmax - xmin) * density.size()), (int) density.size()-1);
    int yindex = std::min((int) ((vy[i] - ymin) / (ymax - ymin) * density[0].size()), (int) density[0].size()-1);

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
}

void densityMap(std::vector< std::vector<int> > &density, int &maxDensity,  std::string outputName) {
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
