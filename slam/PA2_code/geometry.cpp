/*
 * geometry.cpp
 * Copyright (C) 2018 exbot <exbot@ubuntu>
 *
 * Distributed under terms of the MIT license.
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv){

		Isometry3d T_cw1 = Isometry3d::Identity();
		Isometry3d T_cw2 = Isometry3d::Identity();

		Quaterniond q_cw1(0.55, 0.3, 0.2, 0.2);
		Quaterniond q_cw2(-0.1, 0.3, -0.7, 0.2);

		Vector3d t_cw2(-0.1,0.4,0.8);
		Vector3d t_cw1(0.7,1.1,0.2);
		Vector3d p_c1(0.5,-0.1,0.2);

		T_cw1.rotate(q_cw1.normalized().toRotationMatrix());
		T_cw1.pretranslate(t_cw1);
		T_cw2.rotate(q_cw2.normalized().toRotationMatrix());
		T_cw2.pretranslate(t_cw2);

		Vector3d p_c2 = T_cw2 * T_cw1.inverse() * p_c1;
		cout << "P in camera2: "<< endl << p_c2 << endl;
return 1;
}
