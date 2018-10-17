#include <iostream>
#include <ctime>
using namespace std;

//eigen 
#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50

int main(char argc, char **argv){
	
	Eigen::Matrix<float,2,3> matrix_23;
	Eigen::Vector3d v_3d;
	Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
	Eigen::Matrix< double,Eigen::Dynamic,Eigen::Dynamic> matrix_dynamic;
	Eigen::MatrixXd matrix_x;
	
	matrix_23 << 1,1,2,2,5,3;
	cout << matrix_23 <<endl;
	
return 0;
}
