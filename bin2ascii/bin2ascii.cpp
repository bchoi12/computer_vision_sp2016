#include <byteswap.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#define DEBUG 1
#define ASCII_FORMAT "ascii 1.0"
#define END_HEADER "end_header"

void split(const std::string &s, std::vector<std::string> &elems);
void writeASCII(std::ifstream &ifs, std::ofstream &ofs, std::vector<std::string> &properties);
void writeProperty(std::ifstream &ifs, std::ofstream &ofs, std::string &type, unsigned int &output);

std::map<std::string, char> initSizes() {
  std::map<std::string, char> m;
  m["char"] = 1;
  m["uchar"] = 1;
  m["short"] = 2;
  m["ushort"] = 2;
  m["int"] = 4;
  m["uint"] = 4;
  m["float"] = 4;
  m["double"] = 8;
  return m;
}

std::map<std::string, char> sizes = initSizes();

int main(int argc, char** argv) {
  if (argc < 2) {
    printf("Usage: bin2ascii fname\n  fname: input ply in binary big endian format\n");
    return 1;
  }

  std::string inputFile = argv[1];
  if (inputFile.substr(inputFile.find_last_of('.')+1) != "ply") {
    printf("Error: expected ply file\n");
    return 2;
  }

  std::ifstream ifs(inputFile.c_str(), std::ios::binary);

  if (ifs.is_open()) {

    std::string outputFile = inputFile.substr(0, inputFile.find_last_of('.')) + "_ascii.ply";
    std::ofstream ofs(outputFile.c_str());
    
    if (DEBUG) {
      printf("Converting %s to %s...\n", inputFile.c_str(), outputFile.c_str());
    }

    std::string line;
    for(int i=0; i<2; i++) {
      std::getline(ifs, line);
    }
    
    ofs << "ply" << std::endl << "format " << ASCII_FORMAT << std::endl;
    
    std::vector<std::string> vertexProperties, faceProperties, edgeProperties;
    unsigned int vertices = 0, faces = 0, edges = 0, state = 0;

    // read header lines
    do {
      std::getline(ifs, line);
      ofs << line << std::endl;
    
      std::vector<std::string> tokens;
      split(line, tokens);
    
      if (tokens[0] == "element") {
        if (tokens[1] == "vertex") {
          vertices = atoi(tokens[2].c_str());
	  state = 1;
	} else if (tokens[1] == "face") {
	  faces = atoi(tokens[2].c_str());
	  state = 2;
	} else if (tokens[1] == "edge") {
	  edges = atoi(tokens[2].c_str());
	  state = 3;
	}
      } else if (tokens[0] == "property") {
	if (state == 1) {
	  vertexProperties.push_back(tokens[1]);
	} else if (state == 2) {
	  faceProperties.push_back(tokens[1]);
	  faceProperties.push_back(tokens[2]);
	  faceProperties.push_back(tokens[3]);
	} else if (state == 3) {
	  edgeProperties.push_back(tokens[1]);
	}
      }
    }
    while(line != END_HEADER);

    // convert binary data and write to output file
    for(unsigned int i=0; i<vertices; i++) {
      writeASCII(ifs, ofs, vertexProperties);
    }
    for(unsigned int i=0; i<faces; i++) {
      writeASCII(ifs, ofs, faceProperties);
    }
    for(unsigned int i=0; i<edges; i++) {
      writeASCII(ifs, ofs, edgeProperties);
    }
    ofs.close();
  } else {
    printf("Error opening file\n");
    return 3;
  }

  ifs.close();

  printf("Finished!\n");
  
  return 0;
}

// split a string by whitespace
void split(const std::string &s, std::vector<std::string> &elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ' ')) {
    elems.push_back(item);
  }
}

// writes a line based on property types
void writeASCII(std::ifstream &ifs, std::ofstream &ofs, std::vector<std::string> &properties) {
  if (properties.size() == 0) {
    return;
  }

  unsigned int output = -1;
  
  if (properties[0] == "list") {
    unsigned int listSize  = -1;
    writeProperty(ifs, ofs, properties[1], listSize);

    for(unsigned int i=0; i<listSize; i++) {
      ofs << " ";
      writeProperty(ifs, ofs, properties[2], output);
    }
  } else {
    for(unsigned int i=0; i<properties.size(); i++) {
      if (i > 0) {
	ofs << " ";
      }
      writeProperty(ifs, ofs, properties[i], output);
    }
  }

  // add new line
  ofs << std::endl;
}

// writes a single property to the output stream 
void writeProperty(std::ifstream &ifs, std::ofstream &ofs, std::string &type, unsigned int &output) {

  unsigned int size = sizes[type];
  char buffer[size];
  ifs.read((char *) &buffer, sizeof buffer);

  // change endianness
  for(unsigned int i=0; i<size/2; i++) {
    buffer[i] = buffer[i] ^ buffer[size-i-1];
    buffer[size-i-1] = buffer[i] ^ buffer[size-i-1];
    buffer[i] = buffer[i] ^ buffer[size-i-1];
  }
  
  if (type == "char") {
    output = (int) *reinterpret_cast<char*>(buffer); // signed??
    ofs << output;
  } else if (type == "uchar") {
    output = (int) *reinterpret_cast<unsigned char*>(buffer);
    ofs << output;
  } else if (type == "short") {
    ofs << *reinterpret_cast<short*>(buffer);
  } else if (type == "ushort") {
    ofs << *reinterpret_cast<unsigned short*>(buffer);
  } else if (type == "int") {
    ofs << *reinterpret_cast<int*>(buffer);
  } else if (type == "uint") {
    ofs << *reinterpret_cast<unsigned int*>(buffer);
  } else if (type == "float") {
    ofs << *reinterpret_cast<float*>(buffer);
  } else if (type == "double") {
    ofs << *reinterpret_cast<double*>(buffer);
  } else {
    printf("ERROR: unknown type %s!\n", type.c_str());
  }
}
