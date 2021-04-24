#!/bin/bash
mkdir -p odo_sweep_images/{00,01,02,04,05,06,07,08,09,10}
for i in {0000..4540}; do ../bin/plot_semantic_image odo_sweep_coordinates/00/$i.txt $SEMANTICKITTI/dataset/sequences/00/labels/00$i.label odo_sweep_images/00/$i.ppm ; convert odo_sweep_images/00/$i.ppm odo_sweep_images/00/$i.png; rm odo_sweep_images/00/$i.ppm; done
for i in {0000..1100}; do ../bin/plot_semantic_image odo_sweep_coordinates/01/$i.txt $SEMANTICKITTI/dataset/sequences/01/labels/00$i.label odo_sweep_images/01/$i.ppm ; convert odo_sweep_images/01/$i.ppm odo_sweep_images/01/$i.png; rm odo_sweep_images/01/$i.ppm; done
for i in {0000..4660}; do ../bin/plot_semantic_image odo_sweep_coordinates/02/$i.txt $SEMANTICKITTI/dataset/sequences/02/labels/00$i.label odo_sweep_images/02/$i.ppm ; convert odo_sweep_images/02/$i.ppm odo_sweep_images/02/$i.png; rm odo_sweep_images/02/$i.ppm; done
for i in {0000..0270}; do ../bin/plot_semantic_image odo_sweep_coordinates/04/$i.txt $SEMANTICKITTI/dataset/sequences/04/labels/00$i.label odo_sweep_images/04/$i.ppm ; convert odo_sweep_images/04/$i.ppm odo_sweep_images/04/$i.png; rm odo_sweep_images/04/$i.ppm; done
for i in {0000..2760}; do ../bin/plot_semantic_image odo_sweep_coordinates/05/$i.txt $SEMANTICKITTI/dataset/sequences/05/labels/00$i.label odo_sweep_images/05/$i.ppm ; convert odo_sweep_images/05/$i.ppm odo_sweep_images/05/$i.png; rm odo_sweep_images/05/$i.ppm; done
for i in {0000..1100}; do ../bin/plot_semantic_image odo_sweep_coordinates/06/$i.txt $SEMANTICKITTI/dataset/sequences/06/labels/00$i.label odo_sweep_images/06/$i.ppm ; convert odo_sweep_images/06/$i.ppm odo_sweep_images/06/$i.png; rm odo_sweep_images/06/$i.ppm; done
#for i in {00??..110?}; do ../bin/plot_semantic_image odo_sweep_coordinates/07/$i.txt $SEMANTICKITTI/dataset/sequences/07/labels/00$i.label odo_sweep_images/07/$i.ppm ; convert odo_sweep_images/07/$i.ppm odo_sweep_images/07/$i.png; rm odo_sweep_images/07/$i.ppm; done
for i in {1100..5170}; do ../bin/plot_semantic_image odo_sweep_coordinates/08/$i.txt $SEMANTICKITTI/dataset/sequences/08/labels/00$i.label odo_sweep_images/08/$i.ppm ; convert odo_sweep_images/08/$i.ppm odo_sweep_images/08/$i.png; rm odo_sweep_images/08/$i.ppm; done
for i in {0000..1590}; do ../bin/plot_semantic_image odo_sweep_coordinates/09/$i.txt $SEMANTICKITTI/dataset/sequences/09/labels/00$i.label odo_sweep_images/09/$i.ppm ; convert odo_sweep_images/09/$i.ppm odo_sweep_images/09/$i.png; rm odo_sweep_images/09/$i.ppm; done
for i in {0000..1200}; do ../bin/plot_semantic_image odo_sweep_coordinates/10/$i.txt $SEMANTICKITTI/dataset/sequences/10/labels/00$i.label odo_sweep_images/10/$i.ppm ; convert odo_sweep_images/10/$i.ppm odo_sweep_images/10/$i.png; rm odo_sweep_images/10/$i.ppm; done