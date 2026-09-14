set terminal pngcairo size 1000,600 enhanced font 'Sans,12'
set output 'assignment1_path.png'
set datafile separator ','
set title 'Projectile trajectory'
set xlabel 'Horizontal position (m)'
set ylabel 'Vertical position (m)'
set grid
set yrange [0:*]
set key top right
plot 'assignment1_results.csv' every ::1 using 2:4 with lines lw 2 lc rgb '#0072B2' title 'Continuous (RK4)', \
     '' every 12::1 using 6:8 with points pt 6 ps 1 lc rgb '#D55E00' title 'Discrete (Taylor)'
unset output
