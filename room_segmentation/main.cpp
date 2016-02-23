#include <stdio.h>
#include <stdlib.h>

#include <sstream>
#include <string>

#include "segment.h"

#define OPEN_FREESPACE 1
#define CLOSE_WALLS 0
#define TEST_VISIBILITY 0

void test(Segment &segment, int start, int end);

int main(int argc, char** argv) {
  if (argc < 2) {
    printf("Usage: segment fname\n  fname: input ply in ascii format");
    return 1;
  }
  
  Segment segment(argv[1]);

  segment.computeDensity();
  segment.findWalls();

  segment.densityMap(segment.density, "density_map");
  segment.binaryMap(segment.walls, "wall_map");
  segment.binaryMap(segment.freeSpace, "freespace_map");

  if (OPEN_FREESPACE) {
      segment.open(segment.freeSpace);
      segment.binaryMap(segment.freeSpace, "opened_freespace_map");
  }

  if (CLOSE_WALLS) {
    segment.close(segment.walls);
    segment.binaryMap(segment.walls, "closed_wall_map");
  }

  segment.subsample();

  if (TEST_VISIBILITY) {
    test(segment, 200, 202);
    test(segment, 350, 352);
    test(segment, 510, 525);
  }
  
  segment.computeFreeSpaceVisibility();
  segment.clustering();
  
  printf("Finished running segmentation\n");
  
}

// careful may throw out of bounds
void test(Segment &segment, int start, int end) {
  for(int i=start; i<=end; i++) {
    std::ostringstream cc;
    cc << "test_map" << i;
    segment.testVisibility(i, cc.str());
  }
}
