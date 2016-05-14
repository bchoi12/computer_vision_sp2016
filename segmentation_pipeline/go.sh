#!/bin/bash

if [ "$#" -ne 3 ]; then
	echo "Usage: go.sh ply name width"
	echo "  ply: path of ply file in ascii format"
	echo "  name: name used when writing output images"
	echo "  width: width in pixels of images before rotation"
	exit 1
fi

if [ ! -d "$2_output" ]; then
	echo "Creating new output directory..."
	mkdir $2_output
fi

./image/image $1 $3 $2
./rotate/rotate $2
./mrf/mrf $2

cp $2_freespace_mrf.png rpca/

cd rpca
matlab -r "rpca '$2'" -nodisplay -nojvm -nodesktop
cd ..

cp rpca/$2_freespace_rpca.png $2_freespace_rpca.png

./segment/segment $2

convert $2_cluster_map.ppm $2_cluster_map.png

mv $2_*.png $2_output/
mv $2_*.ppm $2_output/
