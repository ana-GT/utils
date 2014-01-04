#!/bin/sh

# Find all .dae files and convert them to .ply
for i in *.dae; do
	filename=$(basename "$i" .dae )
	meshlabserver -i "$i" -o "${filename}.ply" -s meshlabScale.mlx
done
