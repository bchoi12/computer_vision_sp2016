#include <stdio.h>
#include <stdlib.h>

#include <string>

#include "segment.h"

int main(int argc, char** argv) {

  std::string name = "";
  if (argc > 1) {
    name = "_" + std::string(argv[1]);
  }
  
  Segment segment(name);

  segment.subsample();
  segment.computeFreeSpaceVisibility();
  segment.clustering();
  
  printf("Finished running segmentation\n");
  
}