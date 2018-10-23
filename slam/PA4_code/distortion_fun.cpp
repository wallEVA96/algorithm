/*
 * distortion_fun.cpp
 *                        _ _                 
 *         __      ____ _| | | _____   ____ _ 
 *         \ \ /\ / / _` | | |/ _ \ \ / / _` |
 *          \ V  V / (_| | | |  __/\ V / (_| |
 *           \_/\_/ \__,_|_|_|\___| \_/ \__,_|
 * Copyright (C) 2018 walleva <walleva@ubuntu>
 *
 * Distributed under terms of the MIT license.
 */

#include <iostream>
#include <fstream>

/*
 * use GNUPLOT to plot distort image;
 * 
 */

using namespace std;
string data_path = "/home/walleva/algorithm/slam/PA4_code/gnuplot.txt";

int main(int argc, char **argv){
	ofstream of_data(data_path.c_str());
//	int v = 11,u=22;
	if(!of_data.is_open()){
		cout << "open data failed" << endl;
		return -1;
	}

	double k1=-0.28,k2=0.074,fx=458.654,fy=457.296,p1=0.00019359, p2=1.76187114e-05;
	// p1 and p2 is for undistroting tangential. ----()---->
	for (int v=-200; v<200; v++)
		for (int u=-200; u<200; u++){
			double u_temp = u/fx;
			double v_temp = v/fy; 
			double r2 = u_temp*u_temp+v_temp*v_temp;
			double u_dis = u_temp*(1+k1*r2+k2*r2*r2)+2*p1*u_temp*v_temp+p2*(r2+2*u_temp*u_temp);
			double v_dis = v_temp*(1+k1*r2+k2*r2*r2)+2*p2*u_temp*v_temp+p1*(r2+2*v_temp*v_temp);
			//double u_dis = u_temp+2*p1*u_temp*v_temp+p2*(r2+2*u_temp*u_temp);
			//double v_dis = v_temp+2*p2*u_temp*v_temp+p1*(r2+2*v_temp*v_temp);
			of_data << u_dis*fx << " " << v_dis*fy << endl;
	}
	of_data.close();

return 0;
}

