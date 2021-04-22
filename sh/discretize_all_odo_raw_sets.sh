#!/bin/bash

mkdir -p odo_sweep_coordinates/{00,01,02,04,05,06,07,08,09,10}
#for i in {0003..4543}; do ../bin/discretize_sweep $KITTI/2011_10_03/2011_10_03_drive_0027_extract/velodyne_points/data/000000$i.txt odo_sweep_coordinates/00/$(printf %04d $(("10#$i-3"))).txt; done
#for i in {0004..1104}; do ../bin/discretize_sweep $KITTI/2011_10_03/2011_10_03_drive_0042_extract/velodyne_points/data/000000$i.txt odo_sweep_coordinates/01/$(printf %04d $(("10#$i-4"))).txt; done
#for i in {0003..4663}; do ../bin/discretize_sweep $KITTI/2011_10_03/2011_10_03_drive_0034_extract/velodyne_points/data/000000$i.txt odo_sweep_coordinates/02/$(printf %04d $(("10#$i-3"))).txt; done
for i in {0003..0273}; do ../bin/discretize_sweep $KITTI/2011_09_30/2011_09_30_drive_0016_extract/velodyne_points/data/000000$i.txt odo_sweep_coordinates/04/$(printf %04d $(("10#$i-3"))).txt; done
#for i in {0003..2763}; do ../bin/discretize_sweep $KITTI/2011_09_30/2011_09_30_drive_0018_extract/velodyne_points/data/000000$i.txt odo_sweep_coordinates/05/$(printf %04d $(("10#$i-3"))).txt; done
#for i in {0003..1103}; do ../bin/discretize_sweep $KITTI/2011_09_30/2011_09_30_drive_0020_extract/velodyne_points/data/000000$i.txt odo_sweep_coordinates/06/$(printf %04d $(("10#$i-3"))).txt; done
#for i in {00??..110?}; do ../bin/discretize_sweep $KITTI/2011_09_30/2011_09_30_drive_0027_extract/velodyne_points/data/000000$i.txt odo_sweep_coordinates/07/$(printf %04d $(("10#$i-3"))).txt; done
#for i in {1103..5173}; do ../bin/discretize_sweep $KITTI/2011_09_30/2011_09_30_drive_0028_extract/velodyne_points/data/000000$i.txt odo_sweep_coordinates/08/$(printf %04d $(("10#$i-1103"))).txt; done
#for i in {0003..1593}; do ../bin/discretize_sweep $KITTI/2011_09_30/2011_09_30_drive_0033_extract/velodyne_points/data/000000$i.txt odo_sweep_coordinates/09/$(printf %04d $(("10#$i-3"))).txt; done
#for i in {0003..1203}; do ../bin/discretize_sweep $KITTI/2011_09_30/2011_09_30_drive_0034_extract/velodyne_points/data/000000$i.txt odo_sweep_coordinates/10/$(printf %04d $(("10#$i-3"))).txt; done
