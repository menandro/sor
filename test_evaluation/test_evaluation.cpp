#pragma once

#include "lib_link.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opensor_reconflow/Reconflow.h>
#include <opencv2/optflow.hpp>
#include <opensor_camerapose/CameraPose.h>
#include <opensor_kitti/Kitti.h>
#include "test_evaluation.h"
#include <string>
#include <vector>

void getFilenames(Filenames *filenames, bool useStereo = true) {
	// Modify these
	filenames->itemNo0 = "0000000007";
	filenames->itemNo1 = "0000000006";
	filenames->mainfolder = "h:/data_kitti_raw/2011_09_26/2011_09_26_drive_0093_sync/";
	filenames->cameramatrix = "h:/data_kitti_raw/2011_09_26/2011_09_26_calib/2011_09_26/calib_cam_to_cam.txt";

	// Output files
	filenames->outputfilename = filenames->mainfolder + "output/output3d";

	// Ground Truth files
	filenames->depthgroundtruth = filenames->mainfolder + "proj_depth/groundtruth/image_02/" + filenames->itemNo0 + ".png";

	// Do not touch these
	filenames->im0filename = filenames->mainfolder + "image_02/data/" + filenames->itemNo0 + ".png";
	filenames->im1filename = filenames->mainfolder + std::string((useStereo) ? "image_03" : "image_02") + "/data/"
		+ std::string((useStereo) ? filenames->itemNo0 : filenames->itemNo1) + ".png";
	filenames->flownetfilename = filenames->mainfolder + std::string((useStereo) ? "flownet_stereo/" : "flownet_02/") 
		+ filenames->itemNo0 + ".flo";
	filenames->depthfilename = filenames->mainfolder + "proj_depth/velodyne_raw/image_02/" + filenames->itemNo0 + ".png";
	//filenames->depthfilename = filenames->mainfolder + "proj_depth/groundtruth/image_02/" + filenames->itemNo0 + ".png";
	
	filenames->outputerror = filenames->mainfolder + "output/" + filenames->itemNo0 + "err.png";
}

// ************************************
// DEFAULT and TESTED parameter settings
// ************************************
void loadParameters(Parameters *params, std::string mode) {
	if (!mode.compare("default")) {
		params->lambda = 50.0f;//50
		params->tau = 0.125f;
		params->alphaTv = 33.3f;//33.3f
		params->alphaFn = 10.0f; //100

		params->alphaProj = 0.01f;//fix to 60 (slight effect) nice:2500.0f

		params->lambdaf = 0.0001f;//0.0001
		params->lambdams = 1.0f;//50
		params->lambdasp = 1.0f; //0.1
								 //final value f=0.01f, ms=5000, sp=10
		params->nWarpIters = 1;
		params->iters = 1000;
		params->scale = 1.1f;
		params->minWidth = 16;
	}

	else if (!mode.compare("nosparseflow")) {
		params->lambda = 50.0f;//50
		params->tau = 0.125f;
		params->alphaTv = 33.3f;//33.3f
		params->alphaFn = 0.0f; //100

		params->alphaProj = 0.01f;//fix to 60 (slight effect) nice:2500.0f

		params->lambdaf = 0.0001f;//0.1 (Fdata) //0.00005f //nice:0.0000001f
		params->lambdams = 50.0f;//100 (Fms) nice:0.1f
		params->lambdasp = 0.1f; //nice 1.0f
								 //final value f=0.01f, ms=5000, sp=10
		params->nWarpIters = 1;
		params->iters = 1000;
		params->scale = 1.1f;
		params->minWidth = 16;
	}
}


int test_lidarAs3d() {
	bool useStereo = false;
	bool useLidarAsOpticalFlow = true;
	std::string suffixFor3D = "oursfn";

	Filenames *filenames = new Filenames();
	getFilenames(filenames, useStereo);

	// Main reconstruction pointer
	sor::ReconFlow *flow = new sor::ReconFlow(32, 12, 32);

	// Load Params
	Parameters *params = new Parameters();
	loadParameters(params, "default");

	// Set camera matrices
	Kitti *kitti = new Kitti();
	cv::Mat K, K0, K1;
	kitti->readCalib(filenames->cameramatrix);
	K = kitti->calib->k02;
	if (useStereo) {
		K0 = kitti->calib->k02;
		K1 = kitti->calib->k03;
	}

	// Check image size and compute pyramid nlevels
	cv::Mat iset = cv::imread(filenames->im1filename);
	int width = iset.cols;
	int height = iset.rows;
	int nLevels = 1;
	int pHeight = (int)((float)height / params->scale);
	while (pHeight > params->minWidth) {
		nLevels++;
		pHeight = (int)((float)pHeight / params->scale);
	}
	std::cout << "Pyramid Levels: " << nLevels << std::endl;
	int stride = flow->iAlignUp(width);
	std::cout << "Stride: " << stride << std::endl;
	std::cout << "Width: " << width << std::endl;
	cv::Mat isetpad;
	cv::copyMakeBorder(iset, isetpad, 0, 0, 0, stride - width, cv::BORDER_CONSTANT, 0);

	// Open input images
	cv::Mat i0rgb, i1rgb, flownet, RR, tt;
	i0rgb = cv::imread(filenames->im0filename);
	i1rgb = cv::imread(filenames->im1filename);

	// Open initial matching (flownet)
	flownet = cv::optflow::readOpticalFlow(filenames->flownetfilename);
	if (flownet.empty()) {
		std::cerr << "Flownet file not found." << std::endl;
		return 0;
	}
	else std::cout << "Flownet found." << std::endl;

	// Open initial 3D
	cv::Mat depth, depthMask, Xin, Yin, Zin;
	kitti->readDepth(filenames->depthfilename, depth, depthMask);
	depthTo3d(depth, depthMask, Xin, Yin, Zin, K);

	// Open groundtruth 3D
	cv::Mat depthGt, depthMaskGt;
	kitti->readDepth(filenames->depthgroundtruth, depthGt, depthMaskGt);

	// Solve pose from 3d-2d matches
	cv::Mat R, t;
	if (useStereo) kitti->useStereoPose0203(R, t);
	else solve2d3dPose(i1rgb, Xin, Yin, Zin, depthMask, flownet, K, R, t);
	
	// Convert Depth to Optical Flow
	cv::Mat uLidar, vLidar;
	depthToOpticalFlow(depth, depthMask, uLidar, vLidar, K, R, t);

	// Resize images by padding
	cv::Mat uLidarpad, vLidarpad;
	cv::copyMakeBorder(uLidar, uLidarpad, 0, 0, 0, stride - width, cv::BORDER_CONSTANT, 0);
	cv::copyMakeBorder(vLidar, vLidarpad, 0, 0, 0, stride - width, cv::BORDER_CONSTANT, 0);
	cv::Mat Xinpad, Yinpad, Zinpad, depthMaskpad;
	cv::Mat i1rgbpad, i0rgbpad, flownetpad;
	cv::copyMakeBorder(i0rgb, i0rgbpad, 0, 0, 0, stride - width, cv::BORDER_CONSTANT, 0);
	cv::copyMakeBorder(i1rgb, i1rgbpad, 0, 0, 0, stride - width, cv::BORDER_CONSTANT, 0);
	cv::copyMakeBorder(flownet, flownetpad, 0, 0, 0, stride - width, cv::BORDER_CONSTANT, 0);
	cv::copyMakeBorder(Xin, Xinpad, 0, 0, 0, stride - width, cv::BORDER_CONSTANT, 0);
	cv::copyMakeBorder(Yin, Yinpad, 0, 0, 0, stride - width, cv::BORDER_CONSTANT, 0);
	cv::copyMakeBorder(Zin, Zinpad, 0, 0, 0, stride - width, cv::BORDER_CONSTANT, 0);
	cv::copyMakeBorder(depthMask, depthMaskpad, 0, 0, 0, stride - width, cv::BORDER_CONSTANT, 0);
	cv::Mat flownet2[2];   //split flownet channels
	cv::split(flownetpad, flownet2);

	// Initialize ReconFlow
	flow->initializeR(width, height, 3, nLevels, params->scale, sor::ReconFlow::METHODR_TVL1_MS_FNSPARSE_LIDAR,
		params->lambda, 0.0f, params->lambdaf, params->lambdams, params->lambdasp,
		params->alphaTv, params->alphaProj, params->alphaFn,
		params->tau, params->nWarpIters, params->iters);
	flow->setCameraMatrices(K, K);

	// Copy data to GPU
	flow->copyImagesToDevice(i0rgbpad, i1rgbpad);

	if (useLidarAsOpticalFlow)
		flow->copySparseOpticalFlowToDevice(uLidarpad, vLidarpad, depthMaskpad);
	else
		flow->copySparseOpticalFlowToDevice(flownet2[0], flownet2[1], createFlownetMask(flownet2[0])); //can set a mask as third argument

	flow->copySparse3dToDevice(Xinpad, Yinpad, Zinpad, depthMaskpad);

	// Calculate ReconFlow
	flow->solveR(R, t, 100.0f);

	// Copy GPU results to CPU
	// Initialize handler matrices for display and output
	cv::Mat uvrgb = cv::Mat(isetpad.size(), CV_32FC3);
	cv::Mat u = cv::Mat(isetpad.size(), CV_32F);
	cv::Mat v = cv::Mat(isetpad.size(), CV_32F);
	cv::Mat X = cv::Mat(isetpad.size(), CV_32F);
	cv::Mat Y = cv::Mat(isetpad.size(), CV_32F);
	cv::Mat Z = cv::Mat(isetpad.size(), CV_32F);
	flow->copyOpticalFlowToHost(u, v, uvrgb);
	flow->copy3dToHost(X, Y, Z);

	// Save output 3D as ply file
	std::vector<cv::Vec3b> colorbuffer(stride*height);
	cv::Mat colorMat = cv::Mat(static_cast<int>(colorbuffer.size()), 1, CV_8UC3, &colorbuffer[0]);
	std::vector<cv::Vec3f> buffer(stride*height);
	cv::Mat cloudMat = cv::Mat(static_cast<int>(buffer.size()), 1, CV_32FC3, &buffer[0]);
	colorbuffer.clear();
	buffer.clear();

	for (int j = 0; j < height; j++) {
		for (int i = 0; i < stride; i++) {
			cv::Vec3b rgb = i0rgbpad.at<cv::Vec3b>(j, i);
			colorbuffer.push_back(rgb);

			float x = X.at<float>(j, i);
			float y = Y.at<float>(j, i);
			float z = Z.at<float>(j, i);
			//buffer.push_back(cv::Vec3f(x, -y, -z));
			if (((z <= 80) && (z >= 4)) && ((x <= 100) && (y > -100)) && ((y <= 100) && (y > -100))) {
				buffer.push_back(cv::Vec3f(x, -y, -z));
			}
			else {
				x = std::numeric_limits<float>::quiet_NaN();
				y = std::numeric_limits<float>::quiet_NaN();
				z = std::numeric_limits<float>::quiet_NaN();
				buffer.push_back(cv::Vec3f(x, y, z));
			}
		}
	}
	cv::viz::WCloud cloud(cloudMat, colorMat);

	std::ostringstream output3d;
	std::cout << filenames->outputfilename << suffixFor3D << ".ply" << std::endl;
	output3d << filenames->outputfilename << suffixFor3D << ".ply";
	cv::viz::writeCloud(output3d.str(), cloudMat, colorMat);

	//cv::imshow("Z", Z);
	/*cv::imwrite("h:/data_kitti_raw/2011_09_26/2011_09_26_drive_0093_sync/output/Z.png", Z * 10);
	cv::imwrite("h:/data_kitti_raw/2011_09_26/2011_09_26_drive_0093_sync/output/Zin.png", Zin * 10);
	cv::imwrite("h:/data_kitti_raw/2011_09_26/2011_09_26_drive_0093_sync/output/depthMask.png", depthMask);*/
	//cv::imshow("Zin", Zin);
	cv::imshow("flow", uvrgb);
	
	// Error
	// Remove padding from Z
	cv::Mat depthOut = Z(cv::Rect(0, 0, width, height));
	cv::Mat depthOut8, depthGt8;
	depthOut.convertTo(depthOut8, CV_8UC1, 5.0f);
	depthGt.convertTo(depthGt8, CV_8UC1, 5.0f);
	cv::imshow("depthOut", depthOut8);
	cv::imshow("depthGT", depthGt8);
	cv::Mat error = cv::Mat::zeros(depthOut.size(), depthOut.type());
	std::vector<float> errors = kitti->depthError(depthOut, depthGt, depthMaskGt, error);
	cv::Mat errorColor;
	kitti->errorToColor(error, depthMaskGt, errorColor);
	cv::imshow("error", errorColor);
	cv::Mat errorRaw;// = cv::imread(filename, cv::IMREAD_UNCHANGED);
	error.convertTo(errorRaw, CV_16U, 256.0);
	imwrite(filenames->outputerror, errorRaw);

	std::cout << "***************************************" << std::endl;
	std::cout << "*************   SCORE  ****************" << std::endl;
	std::cout << "***************************************" << std::endl;
	std::cout << "MAE: " << errors[0] * 1000.0f << std::endl;
	std::cout << "RMSE: " << errors[1] * 1000.0f << std::endl;
	std::cout << "iMAE: " << errors[2] * 1000.0f << std::endl;
	std::cout << "iRMSE: " << errors[3] * 1000.0f << std::endl;
	std::cout << "Log MAE: " << errors[4] << std::endl;
	std::cout << "Log RMSE: " << errors[5] << std::endl;
	std::cout << "Scaled Invariant Metric: " << errors[6] << std::endl;
	std::cout << "Abs. Relative: " << errors[7] << std::endl;
	std::cout << "Squared Relative: " << errors[8] << std::endl;
	std::cout << "***************************************" << std::endl;

	cv::waitKey();
	return 0;
}

int test_lidarAs3dFlownet() {
	return 0;
}

int main(int argc, char **argv)
{
	if (findCudaDevice(argc, (const char **)argv) == 0) {
		test_lidarAs3d();
		//test_sparseFlownet();
		//test_sparseLidar();
		//test_withoutFlownet();
		//test_twoFrameOpticalFlow();
	}
	return 0;
}

// ************************
// Utilities
// ************************
cv::Mat createFlownetMask(cv::Mat im) {
	cv::Mat mask = cv::Mat::zeros(im.size(), CV_32F);
	int height = im.rows;
	int width = im.cols;
	int sparse = 4;
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			if (((i % sparse) == 0) && ((j % sparse) == 0)) {
				mask.at<float>(j, i) = 1.0f;
			}
		}
	}
	return mask;
}

void depthTo3d(cv::Mat depth, cv::Mat depthMask, cv::Mat &Xin, cv::Mat &Yin, cv::Mat &Zin, cv::Mat K) {
	// Convert depth to 3D points
	Xin = cv::Mat::zeros(depthMask.size(), CV_32F);
	Yin = cv::Mat::zeros(depthMask.size(), CV_32F);
	Zin = cv::Mat::zeros(depthMask.size(), CV_32F);
	double centerX = K.at<double>(0, 2);
	double centerY = K.at<double>(1, 2);
	double focalX = K.at<double>(0, 0);
	double focalY = K.at<double>(1, 1);

	int nonZeroCnt = 0;
	for (int j = 0; j < depthMask.rows; j++) {
		for (int i = 0; i < depthMask.cols; i++) {
			if (depthMask.at<float>(j, i) == 1.0f) {
				Zin.at<float>(j, i) = depth.at<float>(j, i);
				Xin.at<float>(j, i) = (float)(((double)i - centerX) * depth.at<float>(j, i) / focalX);
				Yin.at<float>(j, i) = (float)(((double)j - centerY) * depth.at<float>(j, i) / focalY);
				nonZeroCnt++;
			}
		}
	}

	// Save output 3D as ply file
	//std::vector<cv::Vec3b> colorbuffer(nonZeroCnt);
	//cv::Mat colorMat = cv::Mat(static_cast<int>(colorbuffer.size()), 1, CV_8UC3, &colorbuffer[0]);
	std::vector<cv::Vec3f> buffer(nonZeroCnt);
	cv::Mat cloudMat = cv::Mat(static_cast<int>(buffer.size()), 1, CV_32FC3, &buffer[0]);
	//colorbuffer.clear();
	buffer.clear();

	for (int j = 0; j < depthMask.rows; j++) {
		for (int i = 0; i < depthMask.cols; i++) {
			//cv::Vec3b rgb = i0rgbpad.at<cv::Vec3b>(j, i);
			//colorbuffer.push_back(rgb);
			if (depthMask.at<float>(j, i) == 1.0f) {
				float x = Xin.at<float>(j, i);
				float y = Yin.at<float>(j, i);
				float z = Zin.at<float>(j, i);
				buffer.push_back(cv::Vec3f(x, -y, -z));
			}
			/*if (((z <= 100) && (z > 10)) && ((x <= 100) && (y > -100)) && ((y <= 100) && (y > -100))) {
			buffer.push_back(cv::Vec3f(x, -y, -z));
			}
			else {
			x = std::numeric_limits<float>::quiet_NaN();
			y = std::numeric_limits<float>::quiet_NaN();
			z = std::numeric_limits<float>::quiet_NaN();
			buffer.push_back(cv::Vec3f(x, y, z));
			}*/
		}
	}
	//cv::viz::WCloud cloud(cloudMat, colorMat);
	cv::viz::WCloud cloud(cloudMat);

	std::ostringstream output3d;
	output3d << "h:/data_kitti_raw/2011_09_26/2011_09_26_drive_0093_sync/output/lidar3d.ply";
	cv::viz::writeCloud(output3d.str(), cloudMat);

	//cv::imshow("tst", Yin);
	//cv::waitKey();
}

void depthToOpticalFlow(cv::Mat depth, cv::Mat depthMask, cv::Mat &u, cv::Mat &v, cv::Mat K, cv::Mat R, cv::Mat t) {
	// Convert depth to 3D points
	double centerX = K.at<double>(0, 2);
	double centerY = K.at<double>(1, 2);
	double focalX = K.at<double>(0, 0);
	double focalY = K.at<double>(1, 1);
	u = cv::Mat::zeros(depth.size(), CV_32F);
	v = cv::Mat::zeros(depth.size(), CV_32F);

	int nonZeroCnt = 0;
	for (int j = 0; j < depthMask.rows; j++) {
		for (int i = 0; i < depthMask.cols; i++) {
			if (depthMask.at<float>(j, i) == 1.0f) {
				float Z = depth.at<float>(j, i);
				float X = (float)(((double)i - centerX) * depth.at<float>(j, i) / focalX);
				float Y = (float)(((double)j - centerY) * depth.at<float>(j, i) / focalY);

				float X2 = (float)R.at<double>(0, 0)*X + (float)R.at<double>(0, 1)*Y + (float)R.at<double>(0, 2)*Z + (float)t.at<double>(0);
				float Y2 = (float)R.at<double>(1, 0)*X + (float)R.at<double>(1, 1)*Y + (float)R.at<double>(1, 2)*Z + (float)t.at<double>(1);
				float Z2 = (float)R.at<double>(2, 0)*X + (float)R.at<double>(2, 1)*Y + (float)R.at<double>(2, 2)*Z + (float)t.at<double>(2);

				float xproj = (float)focalX*X2 / Z2 + (float)centerX;
				float yproj = (float)focalY*Y2 / Z2 + (float)centerY;

				u.at<float>(j, i) = xproj - (float)i;
				v.at<float>(j, i) = yproj - (float)j;
				nonZeroCnt++;
			}
		}
	}
	//cv::imshow("u", -u);
	//std::vector<cv::Mat> channelForward;
	//channelForward.push_back(u);
	//channelForward.push_back(v);
	//cv::Mat forward;
	//cv::merge(channelForward, forward);
	//cv::optflow::writeOpticalFlow("h:/data_kitti_raw/2011_09_26/2011_09_26_drive_0093_sync/output/flowfromdepth.flo", forward);
}

int solve2d3dPose(cv::Mat im, cv::Mat Xin, cv::Mat Yin, cv::Mat Zin, cv::Mat depthMask, cv::Mat flownet,
	cv::Mat K, cv::Mat &R, cv::Mat &t) {
	/// 1. Extract 2d points from im1 using flownet and depth mask
	cv::Mat nonZeroCoord;
	cv::Mat mask;
	std::vector<cv::Point2f> points2d;
	std::vector<cv::Point3f> points3d;

	depthMask.convertTo(mask, CV_8UC1);
	//std::cout << depthMask.at<float>(188, 648) << std::endl;
	cv::findNonZero(mask, nonZeroCoord);
	for (int i = 0; i < nonZeroCoord.total(); i++) {
		//std::cout << "Zero#" << i << ": " << nonZeroCoord.at<cv::Point>(i).x << ", " << nonZeroCoord.at<cv::Point>(i).y << std::endl;
		float x2d, y2d;
		float x3d, y3d, z3d;
		x2d = nonZeroCoord.at<cv::Point>(i).x + flownet.at<cv::Vec2f>(nonZeroCoord.at<cv::Point>(i))[0]; //x + u
		y2d = nonZeroCoord.at<cv::Point>(i).y + flownet.at<cv::Vec2f>(nonZeroCoord.at<cv::Point>(i))[1]; //y + v
																										 //std::cout << "A#" << i << ": " << nonZeroCoord.at<cv::Point>(i).x << ", " << nonZeroCoord.at<cv::Point>(i).y << std::endl;
																										 //std::cout << "B#" << i << ": " << x2d << ", " << y2d << std::endl;
		points2d.push_back(cv::Point2f(x2d, y2d));

		x3d = Xin.at<float>(nonZeroCoord.at<cv::Point>(i));
		y3d = Yin.at<float>(nonZeroCoord.at<cv::Point>(i));
		z3d = Zin.at<float>(nonZeroCoord.at<cv::Point>(i));
		points3d.push_back(cv::Point3f(x3d, y3d, z3d));
	}

	/*cv::Mat distCoeffs(4, 1, cv::DataType<double>::type);
	distCoeffs.at<double>(0) = 0;
	distCoeffs.at<double>(1) = 0;
	distCoeffs.at<double>(2) = 0;
	distCoeffs.at<double>(3) = 0;*/
	cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

	cv::Mat rvec, tvec;
	cv::solvePnPRansac(points3d, points2d, K, distCoeffs, rvec, t);
	cv::Rodrigues(rvec, R);
	std::cout << "rvec: " << R << std::endl;
	std::cout << "tvec: " << t << std::endl;
	std::cout << "P2D3D size: " << points2d.size() << ":" << points3d.size() << std::endl;
	std::cout << "t: " << (float)t.at<double>(0) << " " << (float)t.at<double>(1) << " " << (float)t.at<double>(2) << std::endl;
	std::cout << "rvec: " << (float)rvec.at<double>(0) << " " << (float)rvec.at<double>(1) << " " << (float)rvec.at<double>(2) << std::endl;
	/*cv::solvePnP(points3d, points2d, K, distCoeffs, rvec, t);
	cv::Rodrigues(rvec, R);
	std::cout << "rvec: " << R << std::endl;
	std::cout << "tvec: " << t << std::endl;*/
	/// 2. Create array of these 2d points and Xin, Yin, Zin
	/// 3. SolvePnP
	return 0;
}
