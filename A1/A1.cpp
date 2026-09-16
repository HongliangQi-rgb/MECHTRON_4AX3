#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace Eigen;

const double g = 9.81;


// Continuous system
Vector4d m_func(double t, const Vector4d& in)
{
    Matrix4d A = Matrix4d::Zero();

    A(0, 1) = 1;
    A(2, 3) = 1;

    Vector4d b;
    b << 0,
         0,
         0,
        -g;

    return A * in + b;
}


// RK4
Vector4d run(double t, const Vector4d& in, double h)
{
    Vector4d k1, k2, k3, k4;

    k1 = m_func(t, in);
    k2 = m_func(t + h / 2, in + h * k1 / 2);
    k3 = m_func(t + h / 2, in + h * k2 / 2);
    k4 = m_func(t + h, in + h * k3);

    return in + h * (k1 + 2*k2 + 2*k3 + k4) / 6;
}


// Discrete system
Vector4d discrete_step(const Vector4d& in, double h)
{
    Matrix4d Ad;

    Ad << 1, h, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, h,
          0, 0, 0, 1;

    Vector4d bd;

    bd << 0,
          0,
         -0.5 * g * h * h,
         -g * h;

    return Ad * in + bd;
}


void solve(const Vector4d& x0, double h)
{
    Vector4d continuous = x0;
    Vector4d discrete = x0;

    std::ofstream file("results.csv");

    file << "time,"
         << "continuous_x,continuous_y,"
         << "discrete_x,discrete_y\n";

    double t = 0;

    while (continuous(2) >= 0)
    {
        file << t << ","
             << continuous(0) << ","
             << continuous(2) << ","
             << discrete(0) << ","
             << discrete(2) << "\n";

        continuous = run(t, continuous, h);
        discrete = discrete_step(discrete, h);

        t = t + h;
    }

    file.close();
}


int main()
{
    double v0 = 20;
    double alpha_deg = 75;
    double h = 0.01;

    double alpha = alpha_deg * M_PI / 180.0;

    Vector4d x0;

    x0 << 0,
          v0 * cos(alpha),
          0,
          v0 * sin(alpha);

    solve(x0, h);


    // Create gnuplot script
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

    plot << "     'results.csv' every 5::1 using 4:5 "
            "with points title 'Discrete'\n";

    plot.close();


    int result = std::system("gnuplot plot.gp");

    if (result != 0)
    {
        std::cout << "gnuplot failed.\n";
        return 1;
    }

    std::cout << "Simulation complete.\n";
    std::cout << "Plot saved to trajectory.png\n";

    return 0;
}