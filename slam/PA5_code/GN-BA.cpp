//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
	string p2_data, p3_data;
	ifstream if_p2(p2d_file.c_str());
	ifstream if_p3(p3d_file.c_str());
 	if( !if_p3.is_open() || !if_p2.is_open() ){
		cout << " p3d.txt or p2d.txt no found!" << endl;
		return -1;
	}
	while( getline(if_p2, p2_data) && getline(if_p3, p3_data) ){
		Vector3d  p3_coor;
		Vector2d  p2_coor;
		istringstream p2_iss(p2_data);
		istringstream p3_iss(p3_data);
		p2_iss >> p2_coor[0] >> p2_coor[1];
		p3_iss >> p3_coor[0] >> p3_coor[1] >> p3_coor[2];
		p3d.push_back(p3_coor);
		p2d.push_back(p2_coor);
	}
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());
	cout << "p3d size:" << p3d.size() << " p2d size:" << p2d.size() << endl;

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti(Matrix3d::Identity(), Vector3d::Zero()); // estimated pose
	for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();
		Vector2d err;
        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
        // compute cost for p3d[I] and p2d[I]	

			Vector3d TP =  T_esti * p3d[i];
			Vector3d pix_coor = K * TP;
			err[0] = p2d[i][0] - pix_coor[0]/pix_coor[2];
			err[1] = p2d[i][1] - pix_coor[1]/pix_coor[2];
			cout << err << endl;
	    // END YOUR CODE HERE

	    // compute jacobian
        	Matrix<double, 2, 6> J;
        // START YOUR CODE HERE
			double x = TP[0], y = TP[1], z = TP[2];
			double x2 = x*x, y2 = y*y, z2 = z*z;
			J << -fx/z, 0, fx*x/z2, fx*x*y/z2, -fx-fx * x2/z2, fx*y/z,
			  	 0, -fy/z, fy*y/z2, fy+fy*y2/z2, -fy*x*y/z2, -fy*x/z;
			cout << J << endl;
		// END YOUR CODE HERE

            H += J.transpose() * J;
          	b += -J.transpose() * err;

			cost += err.transpose() * err;
        }

	// solve dx 
        Vector6d dx;

        // START YOUR CODE HERE 
		dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (std::isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
		T_esti = Sophus::SE3::exp(dx) * T_esti;
        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
