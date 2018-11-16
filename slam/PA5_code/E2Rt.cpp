//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.h>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
	JacobiSVD<MatrixXd> svd(E, ComputeThinU | ComputeThinV);
	Matrix3d U = svd.matrixU();
	Matrix3d V = svd.matrixV();
	Matrix3d d = U.inverse() * E * V.transpose().inverse();
//	Vector3d d = svd.singularValues();
//	Matrix3d dia;
//	dia << d[0], 0,    0,
//			0,   d[1], 0,
//			0,   0,    d[2];
//	cout << " ++++"<< endl << dia << endl;
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;	
	AngleAxisd Rz1 = AngleAxisd(M_PI/2, Vector3d::UnitZ());
	AngleAxisd Rz2 = AngleAxisd(-M_PI/2, Vector3d::UnitZ());
    Matrix3d R1;
    Matrix3d R2;

	t_wedge1 = U * Rz1.matrix() * d * U.transpose();
    cout << "t1 = " << endl << t_wedge1 << endl;
	t_wedge2 = U * Rz2.matrix() * d * U.transpose();
	R1 = U * Rz1.matrix().transpose() * V.transpose();
	R2 = U * Rz2.matrix().transpose() * V.transpose();
    // END YOUR CODE HERE

    cout << "R1 = " << endl << R1 << endl;
    cout << "R2 = " << endl << R2 << endl;
    cout << "t1 = " << endl << Sophus::SO3::vee(t_wedge1) << endl;
    cout << "t2 = " << endl << Sophus::SO3::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    cout << "t^R = " << endl <<  t_wedge1 * R1 << endl;
    cout << "t^R = " << endl <<  t_wedge1 * R2 << endl;
    cout << "t^R = " << endl <<  t_wedge2 * R1 << endl;
    cout << "t^R = " << endl <<  t_wedge2 * R2 << endl;

    return 0;
}
