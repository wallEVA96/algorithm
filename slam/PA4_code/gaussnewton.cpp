//
// Created by 高翔 on 2017/12/15.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
    double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值
    //double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值
    double ae = 1.0, be = 3.0, ce = 5.0;        // 估计参数值
    int N = 100;                                 // 数据点
    double w_sigma = 1.0;                        // 噪声Sigma值
    cv::RNG rng;                                 // OpenCV随机数产生器

    vector<double> x_data, y_data;      // 数据
    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma));
    }

    // 开始Gauss-Newton迭代
    int iterations = 1000;    // 迭代次数
    double cost = 0, lastCost = 0;  // 本次迭代的cost和上一次迭代的cost

    for (int iter = 0; iter < iterations; iter++) {

        Matrix3d H = Matrix3d::Zero();             // Hessian = J^T J in Gauss-Newton
        Vector3d b = Vector3d::Zero();             // bias
        cost = 0;

        for (int i = 0; i < N; i++) {
            double xi = x_data[i], yi = y_data[i];  // 第i个数据点
            // start your code here
            double error = 0;   // 第i个数据点的计算误差
            error = yi - exp(ae * xi * xi + be * xi + ce); // 填写计算error的表达式
            Vector3d J; // 雅可比矩阵
            J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);  // de/da
            J[1] = -xi * exp(ae * xi * xi + be * xi + ce);  // de/db
            J[2] = -exp(ae * xi * xi + be * xi + ce);  // de/dc

	//jocobian矩阵 在只有一个 y的 是 一个 1* 3的矩阵，就跟本实验一样，所以注意 H公式的计算的时候， 行列不要颠倒。
            H += J * J.transpose(); // GN近似的H
            b += -error * J;
            // end your code here

            cost += error * error;
        }

    // 求解线性方程 Hx=b，建议用ldlt
 	// start your code here
	// 原本认为是 下一个点和上一个点进行迭代，但是其实应该是 下一次 所有的点和上一次所有的数据进行迭代。
	// 这样才能利用到最小二乘即最小平方差的判断依据呀。
    // 个人认为很有可能是 单个数据点存在随机性，所以利用总体的数据样本进行 多次迭代逼近最小方差。
	// 但是计算 hessian矩阵和 b矩阵的时候，为何不除以N?
	// 因为 Hdx = b,二者呈线性关系.
	// LM算法可以细化步长。
		Vector3d dx;
		H /= N;
		b /= N; 
		dx = H.ldlt().solve(b);
	// end your code here

        if (std::isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost > lastCost) {
            // 误差增长了，说明近似的不够好
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // 更新abc估计值
        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        lastCost = cost;

        cout << "total cost: " << cost << endl;
    }

    cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;
    return 0;
}
