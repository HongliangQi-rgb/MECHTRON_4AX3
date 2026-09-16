set terminal pngcairo size 1000,600
set output 'trajectory.png'
set datafile separator ','
set title 'Projectile Trajectory'
set xlabel 'Horizontal Position (m)'
set ylabel 'Vertical Position (m)'
set grid
set key top right
plot 'results.csv' every ::1 using 2:3 with lines title 'Continuous (RK4)', \
     'results.csv' every 5::1 using 4:5 with points title 'Discrete'
