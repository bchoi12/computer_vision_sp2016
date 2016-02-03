#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#define END_HEADER "end_header"

void split(const std::string &s, std::vector<std::string> &elems);

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

    // reading vertices hard coded for now
    // expecting 3 floats (x, y, z) at beginning of each line
    
    std::vector<float> vx, vy, vz;
    vx.resize(vertices);
    vy.resize(vertices);
    vz.resize(vertices);
    for(unsigned int i=0; i<vertices; i++) {
      std::getline(ifs, line);
      split(line, tokens);
      vx[i] = atof(tokens[0].c_str());
      vy[i] = atof(tokens[1].c_str());
      vz[i] = atof(tokens[2].c_str());
    }
  }

  ifs.close();
}

// split a string by whitespace
void split(const std::string &s, std::vector<std::string> &elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ' ')) {
    elems.push_back(item);
  }
}
