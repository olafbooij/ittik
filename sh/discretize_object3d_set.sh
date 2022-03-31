#!/bin/bash

zip=$KITTIZIPS/data_object_velodyne.zip

mkdir -p object_sweep_coordinates/training
for framenr in {0000..7481}; do
  unzip $zip training/velodyne/00$framenr.bin
  ../bin/bintotxt  training/velodyne/00$framenr.bin training/velodyne/00$framenr.txt
  ../bin/discretize_sweep training/velodyne/00$framenr.txt object_sweep_coordinates/training/00$framenr.bin
  rm training/velodyne/00$framenr.*
done

mkdir -p object_sweep_coordinates/testing
for framenr in {0000..7517}; do
  unzip $zip testing/velodyne/00$framenr.bin
  ../bin/bintotxt  testing/velodyne/00$framenr.bin testing/velodyne/00$framenr.txt
  ../bin/discretize_sweep testing/velodyne/00$framenr.txt object_sweep_coordinates/testing/00$framenr.bin
  rm testing/velodyne/00$framenr.*
done
