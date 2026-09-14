#include <Eigen/Dense>

#include <iomanip>
#include <iostream>
#include <stdexcept>

// Discrete LTI system:
//   x(k+1) = A*x(k) + B*u(k)
//   y(k)   = C*x(k)
// For n states, m inputs and p outputs:
//   A: n x n, B: n x m, C: p x n.
class LTISystem {
public:
    LTISystem(const Eigen::MatrixXd& A,
              const Eigen::MatrixXd& B,
              const Eigen::MatrixXd& C)
        : matrixA(A), matrixB(B), matrixC(C)
    {
        if (A.rows() == 0 || A.rows() != A.cols()) {
            throw std::invalid_argument("A must be a nonempty square matrix.");
        }
        if (B.rows() != A.rows() || B.cols() == 0) {
            throw std::invalid_argument("B must have n rows and at least one column.");
        }
        if (C.cols() != A.rows() || C.rows() == 0) {
            throw std::invalid_argument("C must have n columns and at least one row.");
        }
        stateX = Eigen::VectorXd::Zero(A.rows());
    }

    // Set (or reset) the initial state. Default state is zero.
    void init(const Eigen::VectorXd& x0)
    {
        if (x0.size() != matrixA.rows()) {
            throw std::invalid_argument("Initial state must have n entries.");
        }
        stateX = x0;
    }

    // Advance one discrete time step using u(k).
    void update(const Eigen::VectorXd& u)
    {
        if (u.size() != matrixB.cols()) {
            throw std::invalid_argument("Control input must have m entries.");
        }
        // Evaluate before assignment so all terms use the old state.
        stateX = (matrixA * stateX + matrixB * u).eval();
    }

    Eigen::VectorXd state() const { return stateX; }

    // Output at the current time; this does not advance the simulation.
    Eigen::VectorXd output() const { return matrixC * stateX; }

private:
    Eigen::MatrixXd matrixA;
    Eigen::MatrixXd matrixB;
    Eigen::MatrixXd matrixC;
    Eigen::VectorXd stateX;
};

int main()
{
    try {
        Eigen::MatrixXd A(2, 2);
        Eigen::MatrixXd B(2, 1); // Two states, one input: B is a column.
        Eigen::MatrixXd C(1, 2);
        A << 1.0, 1.0,
             0.0, 0.5;
        B << 0.0,
             1.0;
        C << 1.0, 0.0;

        LTISystem sys(A, B, C);
        Eigen::VectorXd x0(2);
        x0 << 0.0, 0.0;
        sys.init(x0);

        Eigen::VectorXd u(1);
        u << 1.0; // Constant unit input at every step.

        std::cout << "Discrete LTI simulation: x(k+1) = A*x(k) + B*u(k)\n"
                  << "Initial state: [0, 0], constant input: u(k) = 1\n"
                  << "   k          x1          x2           y\n"
                  << std::fixed << std::setprecision(6);
        for (int k = 0; k <= 10; ++k) {
            const Eigen::VectorXd x = sys.state();
            const Eigen::VectorXd y = sys.output();
            std::cout << std::setw(4) << k
                      << std::setw(12) << x(0)
                      << std::setw(12) << x(1)
                      << std::setw(12) << y(0) << '\n';
            if (k < 10) sys.update(u);
        }
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
