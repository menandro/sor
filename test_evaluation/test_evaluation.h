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
	
	//For when K and Rt are on different files (like IVDATA)
	std::string cameraIntrinsic;
	std::string cameraExtrinsic;
	std::string cameraAlignment; //MLP

	std::string outputfilename;
	std::string outputerror;
	std::string outputdepth;
	
	std::string depthgroundtruth;
};

class Parameters {
public:
	float lambda, tau, alphaTv, alphaFn, alphaProj, lambdaf, lambdams, lambdasp, scale;
	int nWarpIters, iters, minWidth;
};

void loadParameters(Parameters *params, std::string mode);

cv::Mat createFlownetMask(cv::Mat im);
void depthTo3d(cv::Mat depth, cv::Mat depthMask, cv::Mat &Xin, cv::Mat &Yin, cv::Mat &Zin, cv::Mat K);
void depthToOpticalFlow(cv::Mat depth, cv::Mat depthMask, cv::Mat &u, cv::Mat &v, cv::Mat K, cv::Mat R, cv::Mat t);
int solve2d3dPose(cv::Mat im, cv::Mat Xin, cv::Mat Yin, cv::Mat Zin, cv::Mat depthMask, cv::Mat flownet,
	cv::Mat K, cv::Mat &R, cv::Mat &t);
