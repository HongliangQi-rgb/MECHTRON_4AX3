#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

class LTISystem
{
private:
    MatrixXd A;
    MatrixXd B;
    MatrixXd C;

    VectorXd x;

public:
    // Constructor
    LTISystem(const MatrixXd& A_,
              const MatrixXd& B_,
              const MatrixXd& C_)
    {
        A = A_;
        B = B_;
        C = C_;
    }

    // Set initial state
    void init(const VectorXd& x0)
    {
        x = x0;
    }

    // Update state: x[k+1] = A*x[k] + B*u[k]
    void update(const VectorXd& u)
    {
        x = A * x + B * u;
    }

    // Return current state
    VectorXd state()
    {
        return x;
    }

    // Return output: y = C*x
    VectorXd output()
    {
        return C * x;
    }
};


int main()
{
    MatrixXd A(2, 2);
    MatrixXd B(2, 1);
    MatrixXd C(1, 2);

    A << 1.0, 0.0,
         2.0, 1.0;

    B << 0.0,
         1.0;

    C << 1.0, 0.0;

    // Create system
    LTISystem sys(A, B, C);

    // Initial state
    VectorXd x0(2);
    x0 << 0.0, 0.0;

    sys.init(x0);

    // Input
    VectorXd u(1);
    u << 1.0;

    // Simulate for 10 steps
    for (int k = 0; k < 10; k++)
    {
        std::cout << "k = " << k << "\n";

        std::cout << "x =\n"
                  << sys.state() << "\n";

        std::cout << "y =\n"
                  << sys.output() << "\n\n";

        sys.update(u);
    }

    return 0;
}