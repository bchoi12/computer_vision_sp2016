#!/bin/bash

folder=$1

for f in $1/*.ppm; do
	convert ./"$f" ./"${f%.ppm}.png"
done
