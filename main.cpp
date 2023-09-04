#include <iostream>
#include <ctime>
#include <vector>
#include <algorithm>
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <eigen3/Eigen/Core> //eigen lib's header folder is in /usr/include/eigen3
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "libHelloSLAM.h"

using namespace Eigen;
using namespace std;

#define MATRIX_SIZE 50

void hello(){
    //hello
    cout << "Hello, SLAM" << endl;
    printHello();
}

void matrix_basics(){
    //1.matrix basics
    Matrix<float, 2, 3> matrix_23f;
    matrix_23f << 1,2,3,4,5,6;
    cout << "matrix_23f: \n" << matrix_23f << endl;
    cout << matrix_23f(0,0) << endl;
    Vector3d v_3d; //Matrix<double, 3, 1> vd_3d;
    v_3d << 3,2,1;
    Matrix<double,2,1> result = matrix_23f.cast<double>() * v_3d; //Matrix Times vector: Type should match
    cout << "matrix_23f * v_3d: \n" << result << endl;
    Matrix3d matrix_33_zero = Matrix3d::Zero(); //Matrix<double, 3, 3>
    Matrix3d matrix_33_random = Matrix3d::Random(); //随机数
    cout << "origin " << matrix_33_random << endl;
    cout << "sum " << matrix_33_random.sum() << endl;
    cout << "transpose " << matrix_33_random.transpose() << endl;
    cout << "trace " << matrix_33_random.trace() << endl;
    cout << "times 10 " << 10 * matrix_33_random << endl;
    cout << "inverse  " << matrix_33_random.inverse() << endl;
    cout << "det " << matrix_33_random.determinant() << endl;
    Matrix<double, Dynamic, Dynamic> matrix_dynamic;
    MatrixXd matrix_x;
    //2.EigenValue and EigenVector: 实对称矩阵可以保证对角化成功
    SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33_random.transpose() * matrix_33_random);
    cout << "Eigen values " << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors " << eigen_solver.eigenvectors() << endl;  
    //3.解方程: matrix_NN * x = v_Nd
    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose(); //保证半正定
    Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);
    //方法一: 直接求逆，运算量大
    clock_t time_stt = clock();
    Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "Time spent: " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x: " << x.transpose() << endl;
    //方法二: 矩阵分解，比如QR分解
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "Time spent: " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x: " << x.transpose() << endl;
    //方法三: 对于正定矩阵还可以用cholesky分解来解方程
    time_stt = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    cout << "Time spent: " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x: " << x.transpose() << endl;
}

void matrix_transformation(){
    //1.旋转矩阵 R
    Matrix3d rotation_matrix = Matrix3d::Identity();
    //2.旋转向量 (旋转向量 => 旋转矩阵)
    AngleAxisd rotation_vector(M_PI/4, Vector3d(0,0,1));
    cout.precision(3);
    cout << "rotation matrix = \n" << rotation_vector.toRotationMatrix() << endl;
    cout << "rotation matrix = \n" << rotation_vector.matrix() << endl;
    rotation_matrix = rotation_vector.matrix();
    //旋转v点
    Vector3d v(1,0,0);
    Vector3d v_rotated = rotation_vector * v;
    cout << "(1,0,0) rotated by angle axis = " << v_rotated.transpose() << endl;
    v_rotated = rotation_matrix * v;
    cout << "(1,0,0) rotated by matrix = " << v_rotated.transpose() << endl;
    //3.欧拉角 (旋转矩阵 => 欧拉角)
    Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0); //ZYX顺序
    cout << "ZYX = " << euler_angles.transpose() << endl;
    //4.欧式变换矩阵 (旋转向量，平移向量 t => 欧式变换矩阵)
    Isometry3d T = Isometry3d::Identity();
    T.rotate(rotation_vector);
    T.pretranslate(Vector3d(1,3,4));
    cout << "Transform matrix = \n" << T.matrix() << endl;
    //变换v点：相当于 R * v + t
    Vector3d v_transformed = T * v;
    cout << "v transformed = " << v_transformed.transpose() << endl;
    //5.四元数 (旋转向量 / 旋转矩阵 => 四元数)
    Quaterniond q = Quaterniond(rotation_vector);
    cout << "quaternion from vector = " << q.coeffs().transpose() << endl; //coeffs (x,y,z,w), w为实部
    q = Quaterniond(rotation_matrix);
    cout << "quaternion from matrix = " << q.coeffs().transpose() << endl; //coeffs (x,y,z,w), w为实部
    //旋转向量
    v_rotated = q * v; //数学上是qvq^(-1)
    cout << "v after rotation" << v_rotated.transpose() << endl;
    cout << "v after rotation" << (q * Quaterniond(0,1,0,0) * q.inverse()).coeffs().transpose()<< endl;
}

void matrix_transformation_remap_coordinate(){
    //旋转
    Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
    q1.normalize();
    q2.normalize();
    //平移
    Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);
    //欧式变换矩阵
    Isometry3d T1w(q1), T2w(q2);
    T1w.pretranslate(t1);
    T2w.pretranslate(t2);
    //点
    Vector3d p1(0.5, 0, 0.2); //point in R1 coordinate -> World coordinate -> R2 coordinate
    Vector3d p2 = T2w * T1w.inverse() * p1;
    cout << p2.transpose() << endl;
}

string trajectory_file = "/home/jfgvl1187/Desktop/SLAM/trajectory.txt";
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);
    for (size_t i = 0; i < poses.size(); i++) {
      // 画每个位姿的三个坐标轴
      Vector3d Ow = poses[i].translation();
      Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
      Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
      Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));
      glBegin(GL_LINES);
      glColor3f(1.0, 0.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Xw[0], Xw[1], Xw[2]);
      glColor3f(0.0, 1.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Yw[0], Yw[1], Yw[2]);
      glColor3f(0.0, 0.0, 1.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Zw[0], Zw[1], Zw[2]);
      glEnd();
    }
    // 画出连线
    for (size_t i = 0; i < poses.size(); i++) {
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = poses[i], p2 = poses[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }
}
int matrix_transformation_visualize_trajectory(){
    //trajectory.txt中记录了一系列的欧式变换矩阵T_WR || 我们要做的就是把机器人坐标系的原点映射到世界坐标系中O_W，即T_WR的平移部分(即T_WR的最后一列), 最后可视化出来的点就是O_W
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
    ifstream fin(trajectory_file);
    if (!fin) {
        cout << "cannot find trajectory file at " << trajectory_file << endl;
        return 1;
    }

    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate(Vector3d(tx, ty, tz));
        poses.push_back(Twr);
    }
    cout << "read total " << poses.size() << " pose entries" << endl;

    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

int main(int argc, char** argv){
    // hello();
    // matrix_basics();
    // matrix_transformation();
    // matrix_transformation_remap_coordinate();
    matrix_transformation_visualize_trajectory();
    return 0;
}