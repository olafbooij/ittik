#!/bin/bash

mkdir -p odo_sweep_coordinates/{00,01,02,04,05,06,07,08,09,10}
for i in {0003..4543}; do ../sh/discretize_zipped_sweep.sh 2011_10_03_drive_0027_extract $i 3    00; done
for i in {0004..1104}; do ../sh/discretize_zipped_sweep.sh 2011_10_03_drive_0042_extract $i 4    01; done
for i in {0003..4663}; do ../sh/discretize_zipped_sweep.sh 2011_10_03_drive_0034_extract $i 3    02; done
for i in {0003..0273}; do ../sh/discretize_zipped_sweep.sh 2011_09_30_drive_0016_extract $i 3    04; done
for i in {0003..2763}; do ../sh/discretize_zipped_sweep.sh 2011_09_30_drive_0018_extract $i 3    05; done
for i in {0003..1103}; do ../sh/discretize_zipped_sweep.sh 2011_09_30_drive_0020_extract $i 3    06; done
#for i in {00??..110?}; do ../sh/discretize_zipped_sweep.sh 2011_09_30_drive_0027_extract $i 3    07; done
for i in {1103..5173}; do ../sh/discretize_zipped_sweep.sh 2011_09_30_drive_0028_extract $i 1103 08; done
for i in {0003..1593}; do ../sh/discretize_zipped_sweep.sh 2011_09_30_drive_0033_extract $i 3    09; done
for i in {0003..1203}; do ../sh/discretize_zipped_sweep.sh 2011_09_30_drive_0034_extract $i 3    10; done
