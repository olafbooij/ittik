unset key
set view 57, 254, 1, 1
set xrange [ -31.6413 : 15.5732 ]
set yrange [ -10.5371 : 30.7755 ]
set term png large
set output "pointcloud.png"
splot "../../../tmp/0000000071.txt" with dots lc "blue"
set output 
#    EOF
