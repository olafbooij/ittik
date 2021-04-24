#!/bin/bash

dataset=$1
framenr=$2
startnr=$3
odonr=$4

date=${dataset:0:10}
unzip $KITTIZIPS/$dataset.zip  $date/$dataset/velodyne_points/data/000000$framenr.txt
../bin/discretize_sweep $date/$dataset/velodyne_points/data/000000$framenr.txt odo_sweep_coordinates/$odonr/$(printf %04d $(("10#$framenr-$startnr"))).bin
rm $date/$dataset/velodyne_points/data/000000$framenr.txt
