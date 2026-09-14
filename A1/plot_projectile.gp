set terminal pngcairo size 1000,700 enhanced font 'Arial,12'
set output 'projectile_trajectory.png'
set title 'Projectile Motion: Continuous vs Discrete'
set xlabel 'Horizontal position p_x (m)'
set ylabel 'Vertical position p_y (m)'
set grid
set key top right
plot 'continuous.dat' using 2:3 with lines linewidth 3 title 'Continuous (RK4)', \
     'discrete.dat' using 2:3 every 10 with points pointtype 7 pointsize 0.7 title 'Discrete'
