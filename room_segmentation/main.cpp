#include <stdio.h>
#include <stdlib.h>

#include "segment.h"

#define OPEN_FREESPACE 1
#define CLOSE_WALLS 1

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
    segment.binaryMap(segment.walls, "opened_wall_map");
  }

  printf("Finished running segmentation\n");
  
}
