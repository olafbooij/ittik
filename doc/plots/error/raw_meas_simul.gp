unset key
set term svg
set output "raw_meas_simul.svg"
set xlabel "device rotational position in milliradians"
set ylabel "laser ID"
plot [0:50][0:64] "raw_meas_simul" using ($2*1000):1 lc "blue" pt 7 ps .4
set output 
