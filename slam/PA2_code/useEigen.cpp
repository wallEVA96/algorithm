#include <iostream>
#include <ctime>
using namespace std;

//#include <Eigen/Core>
#include <Eigen/SVD>
//#include <Eigen/Dense>
// 稠密矩阵的代数运算 逆矩阵，特征值
#include "/usr/include/eigen3/Eigen/Dense"
#include "/usr/include/eigen3/Eigen/Core"

#define MATRIX_SIZE (50)

int main(int argc, char **argv){
	
	Eigen::Matrix<float,2,3> matrix_23;
	/* Vertor3D is double 3,1, 三维坐标就相当于矩阵中1行3列 */
	Eigen::Vector3d v_3d;
	/* matrxi3d is doule 3,3 */
//	Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
	/* 特殊类型 */
	Eigen::MatrixXd matrix_x;
	
	matrix_23 << 1,1,2,2,5,5;
	cout << matrix_23 <<endl;
	cout << matrix_23.cols() << endl;

	for (int i = 0; i < matrix_23.rows(); i++)
		for(int j = 0;j < matrix_23.cols(); j++)
				cout << matrix_23(i,j) << " ";
	v_3d << 1, 2, 3;
	cout << endl <<  "-- " << v_3d.transpose() << endl;

//	Eigen::Matrix<double,2,3> wrong_type = matrix_23;
//	Eigen::Matrix<double,2,3> cast_type = matrix_23.cast<double>();
	Eigen::Matrix<double,2,1> cast_type = matrix_23.cast<double>() * v_3d;
	cout << cast_type << endl;

	/* 四则运算 */
	Eigen::Matrix3d matrix_3d = Eigen::Matrix3d::Random();
	matrix_3d << 1,2,0,0,2,3,1,0,3;
	cout << "---------------" << endl << matrix_3d << endl;

	cout << "-------transpose--------" << endl << matrix_3d.transpose() << endl;
	cout << "-------sum--------" << endl << matrix_3d.sum() << endl; 
	cout << "-------trace--------" << endl << matrix_3d.trace() << endl;
	cout << "-------multiple--------" << endl << matrix_3d*96 << endl;
	cout << "-------inverse--------" << endl << matrix_3d.inverse() << endl;
	cout << "-------[ 1 ]--------" << endl << matrix_3d.inverse()*matrix_3d << endl;
	cout << "-------determinant------" << endl << matrix_3d.determinant() << endl;

	matrix_3d = Eigen::Matrix3d::Random();
	cout << "-------eigen random:" << endl << matrix_3d << endl;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(matrix_3d.transpose()*matrix_3d);
	cout << "-------eigen tra*mat:" << endl << matrix_3d.transpose()*matrix_3d << endl;
	cout << "-------eigen values:" << endl << solver.eigenvalues() << endl;
	cout << "-------eigen vector" << endl << solver.eigenvectors() << endl;
//	Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> Matrix_MM = Eigen::Matrix<double,MATRIX_SIZE,MATRIX_SIZE>::Random(); //eigen default size is 50
	//Eigen::Matrix<double, 50,50> Matrix_MM = Eigen::Matrix<double,50,50>::Random(); //eigen default size is 50
	//Eigen::Matrix<double, MATRIX_SIZE, 1> v_M = Eigen::Matrix<double,MATRIX_SIZE,1>::Random();
	//cout << "-------Matrix_MM:" << endl << Matrix_MM << endl;
	/* 求逆 */
	clock_t time_st = clock();
//	cout << "-------Result:" << endl << Matrix_MM.inverse()*v_M << endl;
//	cout << "-------Timest:" << endl << 1000*(clock()-time_st)/(double)CLOCKS_PER_SEC <<endl;
//	
//	time_st = clock();
//	cout << "-------Result:" << endl << Matrix_MM.colPivHouseholderQr().solve(v_M) << endl;
//	cout << "-------Result:" << endl << Matrix_MM.fullPivHouseholderQr().solve(v_M) << endl;
//	cout << "-------Timest:" << endl << 1000*(clock()-time_st)/(double)CLOCKS_PER_SEC <<endl;
//	/* 速度没有变快，反而变慢，不懂为何。8ms , 12.1ms*/

	/* 100*100的随机矩阵 */
	Eigen::Matrix<double, 100, 1> v_D = Eigen::Matrix<double,100,1>::Random();
	//Eigen::Matrix< double,Eigen::Dynamic,Eigen::Dynamic> matrix_dynamic;
	Eigen::Matrix< double, 100, 100> matrix_dynamic;
	matrix_dynamic = Eigen::Matrix<double,100,100>::Random();
	time_st = clock();
	cout << "-------Result:" << endl << matrix_dynamic.colPivHouseholderQr().solve(v_D).transpose() << endl;
	cout << "-------Timest:" << endl << 1000*(clock()-time_st)/(double)CLOCKS_PER_SEC <<endl;

	time_st = clock();
	cout << "-------Result:" << endl << matrix_dynamic.ldlt().solve(v_D).transpose() << endl;
	cout << "-------Timest:" << endl << 1000*(clock()-time_st)/(double)CLOCKS_PER_SEC <<endl;

	return 0;
}
