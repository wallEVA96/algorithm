#include <iostream>
#include <ctime>
using namespace std;

#include <Eigen/Core>
// 稠密矩阵的代数运算 逆矩阵，特征值
#include <Eigen/Dense>

#define MATRIX_SIZE 50

int main(char argc, char **argv){
	
	Eigen::Matrix<float,2,3> matrix_23;
	/* Vertor3D is double 3,1, 三维坐标就相当于矩阵中1行3列 */
	Eigen::Vector3d v_3d;
	/* matrxi3d is doule 3,3 */
	Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
	Eigen::Matrix< double,Eigen::Dynamic,Eigen::Dynamic> matrix_dynamic;
	Eigen::MatrixXd matrix_x;
	
	matrix_23 << 1,1,2,2,5,3;
	cout << matrix_23 <<endl;
	
return 0;
}
