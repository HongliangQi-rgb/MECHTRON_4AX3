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

set output 'assignment1_position.png'
set title 'Position versus time'
set xlabel 'Time (s)'
set ylabel 'Position (m)'
set autoscale y
set key top left
plot 'assignment1_results.csv' every ::1 using 1:2 with lines lw 2 lc rgb '#0072B2' title 'x (RK4)', \
     '' every ::1 using 1:4 with lines lw 2 lc rgb '#D55E00' title 'y (RK4)', \
     '' every 12::1 using 1:6 with points pt 6 ps 1 lc rgb '#0072B2' title 'x (Taylor)', \
     '' every 12::1 using 1:8 with points pt 4 ps 1 lc rgb '#D55E00' title 'y (Taylor)'
unset output

set output 'assignment1_velocity.png'
set title 'Velocity versus time'
set xlabel 'Time (s)'
set ylabel 'Velocity (m/s)'
set key bottom left
plot 'assignment1_results.csv' every ::1 using 1:3 with lines lw 2 lc rgb '#0072B2' title 'vx (RK4)', \
     '' every ::1 using 1:5 with lines lw 2 lc rgb '#D55E00' title 'vy (RK4)', \
     '' every 12::1 using 1:7 with points pt 6 ps 1 lc rgb '#0072B2' title 'vx (Taylor)', \
     '' every 12::1 using 1:9 with points pt 4 ps 1 lc rgb '#D55E00' title 'vy (Taylor)'
unset output
