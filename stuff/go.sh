#!/bin/bash

./image/image $1 $3 $2
./rotate/rotate $2
./mrf/mrf $2
./segment/segment $2
