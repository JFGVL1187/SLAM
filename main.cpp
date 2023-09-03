#include <iostream>
#include "libHelloSLAM.h"
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
#define MATRIX_SIZE 50
int main(int argc, char** argv){
    cout << "Hello, SLAM" << endl;
    printHello();

    Matrix<float, 2, 3> matrix_23;


    return 0;
}