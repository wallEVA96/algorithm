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
	/* 特殊类型 */
	Eigen::MatrixXd matrix_x;
	
	matrix_23 << 1,1,2,2,5,5;
	cout << matrix_23 <<endl;
	cout << matrix_23.cols() << endl;

	for (int i = 0; i < matrix_23.rows(); i++)
		for(int j = 0;j < matrix_23.cols(); j++)
				cout << matrix_23(i,j) << " ";
	
return 0;
}
