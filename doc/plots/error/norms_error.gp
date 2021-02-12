unset key
#set term png large
set term svg
set output "tmp/norms_error.svg"
set style data histogram
set style fill solid border 0
set boxwidth 2
set xlabel "laser sorted by vertical angle
set ylabel "mean point distance in mm"
plot [0:65] "tmp/norms_error" using ($1*1000) lc "blue"
set output 
