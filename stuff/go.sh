#!/bin/bash

./image/image $1 $3 $2
./rotate/rotate $2
./mrf/mrf $2

cp freespace_$2_out.png rpca/

cd rpca
matlab -r "rpca '$2'" -nodisplay -nojvm -nodesktop
cd ..

cp rpca/rpca_freespace_$2_out.png freespace_$2_out.png

./segment/segment $2

convert cluster_map_$2.ppm cluster_map_$2.png
