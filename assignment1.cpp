#include <Eigen/Dense>
// 123123
//456456
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

using Vector4 = Eigen::Vector4d;
using Matrix4 = Eigen::Matrix4d;

// Continuous state-space model:
// x = [px, vx, py, vy]^T
// x_dot = A*x + B*g
Vector4 derivative(const Vector4& x, const Matrix4& A,
                   const Vector4& B, double g)
{
    return A * x + B * g;
}

// One fourth-order Runge-Kutta step for the continuous model.
Vector4 rk4Step(const Vector4& x, double h, const Matrix4& A,
                const Vector4& B, double g)
{
    const Vector4 k1 = derivative(x, A, B, g);
    const Vector4 k2 = derivative(x + 0.5 * h * k1, A, B, g);
    const Vector4 k3 = derivative(x + 0.5 * h * k2, A, B, g);
    const Vector4 k4 = derivative(x + h * k3, A, B, g);

    return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

void writeState(std::ofstream& file, double t, const Vector4& x)
{
    // Columns: time, px, py, vx, vy
    file << t << ' ' << x(0) << ' ' << x(2) << ' '
         << x(1) << ' ' << x(3) << '\n';
}

void simulateContinuous(const Vector4& initialState, double h,
                        const Matrix4& A, const Vector4& B, double g)
{
    std::ofstream file("continuous.dat");
    if (!file) {
        throw std::runtime_error("Could not create continuous.dat");
    }

    file << std::fixed << std::setprecision(8);
    file << "# time px py vx vy\n";

    Vector4 x = initialState;
    double t = 0.0;
    writeState(file, t, x);

    // Take at least one step because the initial vertical position is zero.
    do {
        x = rk4Step(x, h, A, B, g);
        t += h;
        writeState(file, t, x);
    } while (x(2) > 0.0);

    std::cout << "Continuous RK4 simulation finished at t = " << t
              << " s, px = " << x(0) << " m\n";
}

void simulateDiscrete(const Vector4& initialState, double h, double g)
{
    // Exact discrete model for constant acceleration over one time step:
    // x[k+1] = Ad*x[k] + Bd*g
    Matrix4 Ad;
    Ad << 1.0, h,   0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, h,
          0.0, 0.0, 0.0, 1.0;

    Vector4 Bd;
    Bd << 0.0,
          0.0,
          -0.5 * h * h,
          -h;

    std::ofstream file("discrete.dat");
    if (!file) {
        throw std::runtime_error("Could not create discrete.dat");
    }

    file << std::fixed << std::setprecision(8);
    file << "# time px py vx vy\n";

    Vector4 x = initialState;
    double t = 0.0;
    writeState(file, t, x);

    do {
        x = Ad * x + Bd * g;
        t += h;
        writeState(file, t, x);
    } while (x(2) > 0.0);

    std::cout << "Discrete simulation finished at t = " << t
              << " s, px = " << x(0) << " m\n";
}

void createGnuplotScript()
{
    std::ofstream file("plot_projectile.gp");
    if (!file) {
        throw std::runtime_error("Could not create plot_projectile.gp");
    }

    file << "set terminal pngcairo size 1000,700 enhanced font 'Arial,12'\n"
         << "set output 'projectile_trajectory.png'\n"
         << "set title 'Projectile Motion: Continuous vs Discrete'\n"
         << "set xlabel 'Horizontal position p_x (m)'\n"
         << "set ylabel 'Vertical position p_y (m)'\n"
         << "set grid\n"
         << "set key top right\n"
         << "plot 'continuous.dat' using 2:3 with lines linewidth 3 "
            "title 'Continuous (RK4)', \\\n"
         << "     'discrete.dat' using 2:3 every 10 with points "
            "pointtype 7 pointsize 0.7 title 'Discrete'\n";
}

int main(int argc, char* argv[])
{
    try {
        constexpr double pi = 3.14159265358979323846;
        constexpr double g = 9.81;

        // Defaults may be replaced with command-line arguments:
        // ./projectile_simulation initial_speed angle_degrees time_step
        const double initialSpeed = (argc > 1) ? std::stod(argv[1]) : 20.0;
        const double angleDegrees = (argc > 2) ? std::stod(argv[2]) : 45.0;
        const double h = (argc > 3) ? std::stod(argv[3]) : 0.01;

        if (initialSpeed <= 0.0) {
            throw std::invalid_argument("Initial speed must be positive.");
        }
        if (angleDegrees <= 0.0 || angleDegrees >= 90.0) {
            throw std::invalid_argument("Angle must be between 0 and 90 degrees.");
        }
        if (h <= 0.0) {
            throw std::invalid_argument("Time step must be positive.");
        }

        const double angleRadians = angleDegrees * pi / 180.0;

        Vector4 initialState;
        initialState << 0.0,
                        initialSpeed * std::cos(angleRadians),
                        0.0,
                        initialSpeed * std::sin(angleRadians);

        Matrix4 A = Matrix4::Zero();
        A(0, 1) = 1.0; // px_dot = vx
        A(2, 3) = 1.0; // py_dot = vy

        Vector4 B;
        B << 0.0, 0.0, 0.0, -1.0; // vy_dot = -g

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Initial speed: " << initialSpeed << " m/s\n"
                  << "Launch angle:  " << angleDegrees << " degrees\n"
                  << "Time step:     " << h << " s\n\n";

        simulateContinuous(initialState, h, A, B, g);
        simulateDiscrete(initialState, h, g);
        createGnuplotScript();

        std::cout << "\nCreated files:\n"
                  << "  continuous.dat\n"
                  << "  discrete.dat\n"
                  << "  plot_projectile.gp\n\n"
                  << "Run this command to create the graph:\n"
                  << "  gnuplot plot_projectile.gp\n";
    }
    catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
