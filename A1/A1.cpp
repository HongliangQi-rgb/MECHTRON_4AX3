#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace Eigen;
const double g = 9.81;

// Continuous system:
// x_dot = A*x + b
Vector4d derivative(const Vector4d& x)
{
    Matrix4d A = Matrix4d::Zero();

    A(0, 1) = 1.0;
    A(2, 3) = 1.0;

    Vector4d b;
    b << 0.0,
         0.0,
         0.0,
        -g;

    return A * x + b;
}


// RK4
Vector4d rk4_step(const Vector4d& x, double h)
{
    Vector4d k1 = derivative(x);
    Vector4d k2 = derivative(x + h * k1 / 2.0);
    Vector4d k3 = derivative(x + h * k2 / 2.0);
    Vector4d k4 = derivative(x + h * k3);

    return x + h * (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0;
}


// Discrete system:
// x[k+1] = Ad*x[k] + bd
Vector4d discrete_step(const Vector4d& x, double h)
{
    Matrix4d Ad;

    Ad << 1.0, h,   0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, h,
          0.0, 0.0, 0.0, 1.0;

    Vector4d bd;

    bd << 0.0,
          0.0,
         -0.5 * g * h * h,
         -g * h;

    return Ad * x + bd;
}


int main()
{
    double v0 = 20.0;
    double alpha_deg = 45.0;
    double h = 0.01;

    double alpha = alpha_deg * M_PI / 180.0;

    Vector4d x0;

    x0 << 0.0,
          v0 * cos(alpha),
          0.0,
          v0 * sin(alpha);

    Vector4d continuous = x0;
    Vector4d discrete = x0;

    // -----------------------------
    // Save simulation data
    // -----------------------------

    std::ofstream file("results.csv");

    file << "time,"
         << "continuous_x,continuous_y,"
         << "discrete_x,discrete_y\n";

    double t = 0.0;

    while (continuous(2) >= 0.0)
    {
        file << t << ","
             << continuous(0) << ","
             << continuous(2) << ","
             << discrete(0) << ","
             << discrete(2) << "\n";

        continuous = rk4_step(continuous, h);
        discrete = discrete_step(discrete, h);

        t += h;
    }

    file.close();

    std::ofstream plot("plot.gp");

    plot << "set terminal pngcairo size 1000,600\n";
    plot << "set output 'trajectory.png'\n";

    plot << "set datafile separator ','\n";

    plot << "set title 'Projectile Trajectory'\n";
    plot << "set xlabel 'Horizontal Position (m)'\n";
    plot << "set ylabel 'Vertical Position (m)'\n";

    plot << "set grid\n";
    plot << "set key top right\n";

    plot << "plot 'results.csv' every ::1 using 2:3 "
            "with lines title 'Continuous (RK4)', \\\n";

    plot << "     'results.csv' every 10::1 using 4:5 "
            "with points title 'Discrete'\n";

    plot.close();


    int result = std::system("gnuplot plot.gp");

    if (result != 0)
    {
        std::cout << "gnuplot failed.\n";
        std::cout << "Make sure gnuplot is installed.\n";

        return 1;
    }

    std::cout << "Simulation complete.\n";
    std::cout << "Data saved to results.csv\n";
    std::cout << "Plot saved to trajectory.png\n";

    return 0;
}