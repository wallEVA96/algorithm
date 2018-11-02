/*
 * lm-cp.cpp
 *                        _ _                 
 *         __      ____ _| | | _____   ____ _ 
 *         \ \ /\ / / _` | | |/ _ \ \ / / _` |
 *          \ V  V / (_| | | |  __/\ V / (_| |
 *           \_/\_/ \__,_|_|_|\___| \_/ \__,_|
 * Copyright (C) 2018 walleva <walleva@ubuntu>
 *
 * Distributed under terms of the MIT license.
 */



nclude <cstdio>
#include <vector>
#include <opencv2/core/core.hpp>
 
using namespace std;
using namespace cv;
 
const double DERIV_STEP = 1e-5;
const int MAX_ITER = 100;
 
 
void LM(double(*Func)(const Mat &input, const Mat ¶ms), // function pointer
				 const Mat &inputs, const Mat &outputs, Mat ¶ms);
 
double Deriv(double(*Func)(const Mat &input, const Mat ¶ms), // function pointer
			 const Mat &input, const Mat ¶ms, int n);
 
// The user defines their function here
double Func(const Mat &input, const Mat ¶ms);
 
int main()
{
	// For this demo we're going to try and fit to the function
	// F = A*exp(t*B)
	// There are 2 parameters: A B
	int num_params = 2;
 
    // Generate random data using these parameters
    int total_data = 8;
 
    Mat inputs(total_data, 1, CV_64F);
    Mat outputs(total_data, 1, CV_64F);
 
	//load observation data
    for(int i=0; i < total_data; i++) {
        inputs.at<double>(i,0) = i+1;  //load year
    }
	//load America population
	outputs.at<double>(0,0)= 8.3;
	outputs.at<double>(1,0)= 11.0;
	outputs.at<double>(2,0)= 14.7;
	outputs.at<double>(3,0)= 19.7;
	outputs.at<double>(4,0)= 26.7;
	outputs.at<double>(5,0)= 35.2;
	outputs.at<double>(6,0)= 44.4;
	outputs.at<double>(7,0)= 55.9;
 
    // Guess the parameters, it should be close to the true value, else it can fail for very sensitive functions!
    Mat params(num_params, 1, CV_64F);
 
	//init guess
    params.at<double>(0,0) = 6;
	params.at<double>(1,0) = 0.3;
 
    LM(Func, inputs, outputs, params);
 
    printf("Parameters from GaussNewton: %lf %lf\n", params.at<double>(0,0), params.at<double>(1,0));
 
    return 0;
}
 
double Func(const Mat &input, const Mat ¶ms)
{
	// Assumes input is a single row matrix
	// Assumes params is a column matrix
 
	double A = params.at<double>(0,0);
	double B = params.at<double>(1,0);
 
	double x = input.at<double>(0,0);
 
    return A*exp(x*B);
}
 
//calc the n-th params' partial derivation ， the params are our  final target
double Deriv(double(*Func)(const Mat &input, const Mat ¶ms), const Mat &input, const Mat ¶ms, int n)
{
	// Assumes input is a single row matrix
 
	// Returns the derivative of the nth parameter
	Mat params1 = params.clone();
	Mat params2 = params.clone();
 
	// Use central difference  to get derivative
	params1.at<double>(n,0) -= DERIV_STEP;
	params2.at<double>(n,0) += DERIV_STEP;
 
	double p1 = Func(input, params1);
	double p2 = Func(input, params2);
 
	double d = (p2 - p1) / (2*DERIV_STEP);
 
	return d;
}
 
void LM(double(*Func)(const Mat &input, const Mat ¶ms),
				 const Mat &inputs, const Mat &outputs, Mat ¶ms)
{
    int m = inputs.rows;
    int n = inputs.cols;
    int num_params = params.rows;
 
    Mat r(m, 1, CV_64F); // residual matrix
	Mat r_tmp(m, 1, CV_64F);
    Mat Jf(m, num_params, CV_64F); // Jacobian of Func()
    Mat input(1, n, CV_64F); // single row input
	Mat params_tmp = params.clone();
 
    double last_mse = 0;
	float u = 1, v = 2;
	Mat I = Mat::ones(num_params, num_params, CV_64F);//construct identity matrix
	int i =0;
    for(i=0; i < MAX_ITER; i++) {
        double mse = 0;
		double mse_temp = 0;
 
        for(int j=0; j < m; j++) {
        	for(int k=0; k < n; k++) {//copy Independent variable vector, the year
        		input.at<double>(0,k) = inputs.at<double>(j,k);
        	}
 
            r.at<double>(j,0) = outputs.at<double>(j,0) - Func(input, params);//diff between previous estimate and observation population
 
            mse += r.at<double>(j,0)*r.at<double>(j,0);
 
            for(int k=0; k < num_params; k++) {
            	Jf.at<double>(j,k) = Deriv(Func, input, params, k);  //construct jacobian matrix
            }
        }
 
        mse /= m;
        params_tmp = params.clone();
		
		Mat hlm = (Jf.t()*Jf + u*I).inv()*Jf.t()*r;
		params_tmp += hlm; 
		for(int j=0; j < m; j++) {
			r_tmp.at<double>(j,0) = outputs.at<double>(j,0) - Func(input, params_tmp);//diff between current estimate and observation population
			mse_temp += r_tmp.at<double>(j,0)*r_tmp.at<double>(j,0);
		}
 
		mse_temp /= m;
 
		Mat q(1,1,CV_64F);
		q = (mse - mse_temp)/(0.5*hlm.t()*(u*hlm-Jf.t()*r));
	    double q_value = q.at<double>(0,0);
		if(q_value>0)
		{
			double s = 1.0/3.0;
			v = 2;
			mse = mse_temp;
			params = params_tmp;
			double temp = 1 - pow(2*q_value-1,3);
			if(s>temp)
			{
				u = u * s;
			}else
			{
				u = u * temp;
			}
		}else
		{
			u = u*v; 
			v = 2*v;
			params = params_tmp;
		}	
		
	
        // The difference in mse is very small, so quit
        if(fabs(mse - last_mse) < 1e-8) {
        	break;
        }
 
        //printf("%d: mse=%f\n", i, mse);
        printf("%d %lf\n", i, mse);
        last_mse = mse;
    }
}
//作者：taotao1233 
//来源：CSDN 
//原文：https://blog.csdn.net/jinshengtao/article/details/53310804 
//版权声明：本文为博主原创文章，转载请附上博文链接！
