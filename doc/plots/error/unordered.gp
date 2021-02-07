unset key
set term svg
set output "unordered.svg"
set xlabel "horizontal angle in milliradians"
set ylabel "vertical angle in milliradians"
plot [0:50][-350:-20] "../../../kitti/2011_09_26/2011_09_26_drive_0002_extract/velodyne_points/data/0000000071.txt" using (atan2($2,$1)*1000):(atan($3/sqrt($1**2+$2**2))*1000) lc "blue" pt 7 ps .4
set output 
