unset key
set view 57, 254, 1, 1
set xrange [ -31.6413 : 15.5732 ]
set yrange [ -10.5371 : 30.7755 ]
set term png large
set output "pointcloud.png"
splot "../../../kitti/2011_09_26/2011_09_26_drive_0002_extract/velodyne_points/data/0000000071.txt" with dots lc "blue"
set output 
#    EOF
