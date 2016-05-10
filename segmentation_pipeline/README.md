Segmentation Pipeline

1. image - reads data from a ply file in ascii format and outputs walls, freespace, and density images
2. rotate - finds the two main orthogonal directions in the image and attempts to rotate the image upright. Does not work if floor map is non-Manhattan or if a lot of noise is present. Outputs rotaetd wall, freespace, and density images.
3. mrf - uses graph cuts (alpha-expansion) to minimize energy and fill small holes and clean up noisy regions in freespace. Outputs cleaned up freespace image.
4. rpca - uses robust PCA to make the freespace image more "blocky," outputs a "blocky" freespace image
5. segment - uses the image from rpca and an image of the walls to apply the room segmentation algorithm, outputs a cluster map of room segmentation results.


