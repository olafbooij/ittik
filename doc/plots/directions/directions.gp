set xlabel "horizontal direction in degrees"
set ylabel "vertical direction in degrees"
unset key
set term svg
set output "directions.svg"
plot [][5:-28] "./directions.xy" using 1:2:0 with labels
set output 
