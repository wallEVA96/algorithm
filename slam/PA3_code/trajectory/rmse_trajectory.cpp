#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string est_file = "/home/walleva/algorithm/slam/PA3_code/trajectory/estimated.txt";
string tru_file = "/home/walleva/algorithm/slam/PA3_code/trajectory/groundtruth.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    /// implement pose reading code
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
		cout << "-- into while" << endl;
		istringstream iss_est(line_data_est);
		istringstream iss_tru(line_data_tru);
		iss_est >> time_est >> t_est[0] >> t_est[1] >> t_est[2] >> q_est.x() >> q_est.y() >> q_est.z() >> q_est.w();
		iss_tru >> time_tru >> t_tru[0] >> t_tru[1] >> t_tru[2] >> q_tru.x() >> q_tru.y() >> q_tru.z() >> q_tru.w();
		SE3_est = Sophus::SE3(q_est, t_est);
		SE3_tru = Sophus::SE3(q_tru, t_tru);
		SE3_err = SE3_tru.inverse() * SE3_est;
		se3_err = SE3_err.log();
		cout << se3_err << endl;
		rmse += se3_err[0]*se3_err[0]+se3_err[1]*se3_err[1]+se3_err[2]*se3_err[2]+
				se3_err[3]*se3_err[3]+se3_err[4]*se3_err[4]+se3_err[5]*se3_err[5];
		size ++;
		cout << "-- " << rmse << "----" << cout << endl;
		//poses.push_back(SE3_err);
	}
	cout << std::sqrt(rmse/size) << endl;
	if( if_estimate.is_open() )
		if_estimate.close();
	if( if_truth.is_open() )
		if_truth.close();
    // draw trajectory in pangolin
    //DrawTrajectory(poses);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}
