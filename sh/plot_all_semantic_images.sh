#!/bin/bash
mkdir -p odo_sweep_images/{00,01,02,04,05,06,07,08,09,10}
for i in {0000..0270}; do ../bin/plot_semantic_image odo_sweep_coordinates/04/$i.txt $SEMANTICKITTI/dataset/sequences/04/labels/00$i.label odo_sweep_images/04/$i.ppm ; done
