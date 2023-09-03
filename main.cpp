#include <iostream>
#include "libHelloSLAM.h"
#include <ctime>
//eigen lib's header folder is in /usr/include/eigen3
#include <Eigen/Core> 
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
#define MATRIX_SIZE 50
int main(int argc, char** argv){
    cout << "Hello, SLAM" << endl;
    printHello();

    Matrix<float, 2, 3> matrix_23f;
    matrix_23f << 1,2,3,4,5,6;
    cout << "matrix_23f: \n" << matrix_23f << endl;
    cout << matrix_23f(0,0) << endl;

    Vector3d v_3d; //Matrix<double, 3, 1> vd_3d;
    v_3d << 3,2,1;
    Matrix<double,2,1> result = matrix_23f.cast<double>() * v_3d;
    cout << "matrix_23f * v_3d: \n" << result.transpose() << endl;

    Matrix3d matrix_33 = Matrix3d::Zero(); //Matrix<double, 3, 3>
    Matrix<double, Dynamic, Dynamic> matrix_dynamic;
    MatrixXd matrix_x;
    MatrixXd matrix_x_linux;

    MatrixXd matrix_x_mac;
    cout << "Mac changed new" << endl;



    return 0;
}