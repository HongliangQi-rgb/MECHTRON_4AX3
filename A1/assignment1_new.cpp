#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>

// State: [pos_x, vel_x, pos_y, vel_y]^T, in SI units.
using State = Eigen::Vector4d;
using Matrix = Eigen::Matrix4d;

constexpr double gravity = 9.81;

// Part 1: continuous model dx/dt = A*x + b:
//     [0 1 0 0]            [ 0 ]
// A = [0 0 0 0],       b = [ 0 ]
//     [0 0 0 1]            [ 0 ]
//     [0 0 0 0]            [-g ]
State derivative(const State& x)
{
    Matrix A = Matrix::Zero();
    A(0, 1) = 1.0;
    A(2, 3) = 1.0;
    const State b(0.0, 0.0, 0.0, -gravity);
    return A * x + b;
}

// Part 2a: integrate the continuous differential equation using RK4.
State rk4_step(const State& x, double h)
{
    const State k1 = derivative(x);
    const State k2 = derivative(x + (h / 2.0) * k1);
    const State k3 = derivative(x + (h / 2.0) * k2);
    const State k4 = derivative(x + h * k3);
    return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

// Part 1: Taylor expansion gives x[k+1] = Ad*x[k] + bd:
//      [1 h 0 0]             [     0     ]
// Ad = [0 1 0 0],       bd = [     0     ]
//      [0 0 1 h]             [-g*h*h / 2 ]
//      [0 0 0 1]             [   -g*h    ]
// Constant acceleration makes this second-order expansion exact at
// sample times (apart from floating-point roundoff).
State discrete_step(const State& x, double h)
{
    Matrix Ad;
    Ad << 1.0, h,   0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, h,
          0.0, 0.0, 0.0, 1.0;
    const State bd(0.0, 0.0, -0.5 * gravity * h * h, -gravity * h);
    return Ad * x + bd;
}

State analytical_solution(const State& initial, double t)
{
    return {initial[0] + initial[1] * t,
            initial[1],
            initial[2] + initial[3] * t - 0.5 * gravity * t * t,
            initial[3] - gravity * t};
}

int main()
{
    // Change these parameters to try other throws. Launch is from ground.
    constexpr double v0 = 20.0;          // m/s
    constexpr double alpha_degrees = 45.0;
    constexpr double h = 0.01;           // seconds
    const double alpha = alpha_degrees * std::acos(-1.0) / 180.0;
    const State initial{0.0, v0 * std::cos(alpha), 0.0, v0 * std::sin(alpha)};
    State continuous = initial;
    State discrete = initial;

    // Include the first sample at or below ground after the flight.
    // No collision or bounce is modeled.
    const double flight_time = 2.0 * initial[3] / gravity;
    const int steps = static_cast<int>(std::ceil(flight_time / h));
    std::ofstream output("assignment1_results.csv");
    if (!output) {
        std::cerr << "Cannot open assignment1_results.csv for writing.\n";
        return 1;
    }
    output << "time,rk4_pos_x,rk4_vel_x,rk4_pos_y,rk4_vel_y,"
              "discrete_pos_x,discrete_vel_x,discrete_pos_y,discrete_vel_y,"
              "exact_pos_x,exact_vel_x,exact_pos_y,exact_vel_y\n";
    output << std::setprecision(16);

    for (int k = 0; k <= steps; ++k) {
        const double t = k * h;
        const State exact = analytical_solution(initial, t);
        output << t;
        for (const State& state : {continuous, discrete, exact})
            for (Eigen::Index i = 0; i < state.size(); ++i)
                output << ',' << state(i);
        output << '\n';

        if (k < steps) {
            continuous = rk4_step(continuous, h);
            discrete = discrete_step(discrete, h);
        }
    }
    output.close();
    if (!output) {
        std::cerr << "Failed to write simulation results.\n";
        return 1;
    }
    // Generate trajectory, position-versus-time, and velocity-versus-time plots.
    std::ofstream plot("assignment1_plot.gp");
    if (!plot) {
        std::cerr << "Cannot create gnuplot script.\n";
        return 1;
    }
    plot << R"(set terminal pngcairo size 1000,600 enhanced font 'Sans,12'
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
)";
    plot.close();
    if (!plot) {
        std::cerr << "Failed to write gnuplot script.\n";
        return 1;
    }
    if (std::system("gnuplot assignment1_plot.gp") != 0) {
        std::cerr << "gnuplot failed. Check that gnuplot is installed.\n";
        return 1;
    }
    std::cout << "Trajectory saved to assignment1_path.png\n"
              << "Position saved to assignment1_position.png\n"
              << "Velocity saved to assignment1_velocity.png\n";
    return 0;
}
