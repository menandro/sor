#pragma once

// Evaluations
// 1. Lidar as 3D (no FlowNet)
int test_lidarAs3d();

// 2. Lidar as 3D + Flownet as 2D
int test_lidarAs3dFlownet();

// 3. Lidar as 3D and 2D
int test_lidarAs3d2d();

// 4. Lidar as 3D and 2D + Flownet as 2D
int test_lidarAs3d2dFlownet2d();

// Utilities
class Filenames {
public:
	std::string itemNo0;
	std::string itemNo1;
	std::string mainfolder;
	std::string im0filename;
	std::string im1filename;
	std::string flownetfilename;
	std::string depthfilename;
	std::string cameramatrix;
	std::string outputfilename;
	std::string outputerror;
	
	std::string depthgroundtruth;
};

class Parameters {
public:
	float lambda, tau, alphaTv, alphaFn, alphaProj, lambdaf, lambdams, lambdasp, scale;
	int nWarpIters, iters, minWidth;
};

class CalibData {
public:
	CalibData() {
		s = std::vector<cv::Mat>(4);
		k = std::vector<cv::Mat>(4);
		d = std::vector<cv::Mat>(4);
		r = std::vector<cv::Mat>(4);
		t = std::vector<cv::Mat>(4);
		sRect = std::vector<cv::Mat>(4);
		rRect = std::vector<cv::Mat>(4);
		pRect = std::vector<cv::Mat>(4);
	};
	std::string date, time;
	float cornerDist;
	std::vector<cv::Mat> s, k, d, r, t, sRect, rRect, pRect;
	cv::Mat k02; //rectified camera matrix of image_02
};

void loadParameters(Parameters *params, std::string mode);
std::vector<float> depthError(cv::Mat depthOut, cv::Mat depthGt, cv::Mat depthMaskGt, cv::Mat &error);

void readCalibKitti(std::string filename, CalibData *calib);
void readDepthKitti(std::string filename, cv::Mat &depth, cv::Mat &mask);
cv::Mat createFlownetMask(cv::Mat im);
void depthTo3d(cv::Mat depth, cv::Mat depthMask, cv::Mat &Xin, cv::Mat &Yin, cv::Mat &Zin, cv::Mat K);
void depthToOpticalFlow(cv::Mat depth, cv::Mat depthMask, cv::Mat &u, cv::Mat &v, cv::Mat K, cv::Mat R, cv::Mat t);
int solve2d3dPose(cv::Mat im, cv::Mat Xin, cv::Mat Yin, cv::Mat Zin, cv::Mat depthMask, cv::Mat flownet,
	cv::Mat K, cv::Mat &R, cv::Mat &t);
