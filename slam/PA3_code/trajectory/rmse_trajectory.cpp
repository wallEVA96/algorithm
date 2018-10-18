#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string est_file = "/home/exbot/algorithm/slam/PA3_code/trajectory/estimated.txt";
string tru_file = "/home/exbot/algorithm/slam/PA3_code/trajectory/groundtruth.txt";

int main(int argc, char **argv) {

    // start your code here (5~10 lines)
	cout << "-- Visual SLAM 4 Chapter " << endl;
	
	/* define variable */
	string input_data_est,line_data_est,
		   input_data_tru,line_data_tru;
	double time_est = 0,time_tru = 0, rmse = 0;
	int size = 0;
	Eigen::Vector3d t_est, t_tru;
	Eigen::Quaterniond q_est, q_tru;
	Sophus::SE3 SE3_err, SE3_est, SE3_tru;
	Eigen::Matrix<double,6,1> se3_err;
	ifstream if_estimate(est_file.c_str());
	ifstream if_truth(tru_file.c_str());

	if( !if_estimate.is_open() || !if_truth.is_open() ){
		cout << "-- trajectory.txt not found" << endl;
		return -1;
	}

	while(getline(if_estimate, line_data_est) &&
		  getline(if_truth,    line_data_tru)){
		istringstream iss_est(line_data_est);
		istringstream iss_tru(line_data_tru);
		iss_est >> time_est >> t_est[0] >> t_est[1] >> t_est[2] >> q_est.x() >> q_est.y() >> q_est.z() >> q_est.w();
		iss_tru >> time_tru >> t_tru[0] >> t_tru[1] >> t_tru[2] >> q_tru.x() >> q_tru.y() >> q_tru.z() >> q_tru.w();
		SE3_est = Sophus::SE3(q_est, t_est);
		SE3_tru = Sophus::SE3(q_tru, t_tru);
		SE3_err = SE3_tru.inverse() * SE3_est;
		se3_err = SE3_err.log();
/*		rmse += se3_err[0]*se3_err[0]+se3_err[1]*se3_err[1]+se3_err[2]*se3_err[2]+
				se3_err[3]*se3_err[3]+se3_err[4]*se3_err[4]+se3_err[5]*se3_err[5];
*/
		rmse += se3_err.transpose()* se3_err;
		size ++;
	}
	cout << "-- RMSE: " << std::sqrt(rmse/size) << endl;
	if( if_estimate.is_open() )
		if_estimate.close();
	if( if_truth.is_open() )
		if_truth.close();
    return 0;
}

