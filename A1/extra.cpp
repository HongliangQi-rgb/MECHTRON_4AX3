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
    LTISystem(const MatrixXd& A_,
              const MatrixXd& B_,
              const MatrixXd& C_)
    {
        A = A_;
        B = B_;
        C = C_;
    }

    void init(const VectorXd& x0)
    {
        x = x0;
    }

    void update(const VectorXd& u)
    {
        x = A * x + B * u;
    }

    VectorXd state()
    {
        return x;
    }

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

    LTISystem sys(A, B, C);
    VectorXd x0(2);
    x0 << 0.0, 0.0;
    sys.init(x0);
    VectorXd u(1);
    u << 1.0;

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