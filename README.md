# Overview

Completely programmatic pipeline that takes ~noisy 3D scans of rooms from a handheld tablet/phone and converts them into a 2D segmented floorplan.

Results:

![results](https://github.com/bchoi12/computer_vision_sp2016/blob/master/segmentation_pipeline/bryan2_output/bryan2_cluster_map.png)

## Developer details

1. bin2ascii - converts ply files from binary format to ascii format
2. data - 3D models of various locations in Washington University in St. Louis, in both binary and ascii format
3. segmentation_pipeline - pipeline used to convert 3D model in ply format to a room-segmented floor map
