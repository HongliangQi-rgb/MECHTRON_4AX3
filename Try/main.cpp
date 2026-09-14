#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Matrix2d mat;
    mat << 1, 2,
           3, 4;
    std::cout << "恭喜你！Eigen 库安装成功！" << std::endl;
    std::cout << "矩阵所有元素的和是: " << mat.sum() << std::endl;
    return 0;
}