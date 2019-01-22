#pragma once
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

//#define LIB_PATH "D:/dev/lib64/"
//#define CV_LIB_PATH "D:/dev/lib64/"
//#define CV_VER_NUM  CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
//
//#ifdef _DEBUG
//#define LIB_EXT "d.lib"
//#else
//#define LIB_EXT ".lib"
//#endif
//#pragma comment(lib, CV_LIB_PATH "opencv_core" CV_VER_NUM LIB_EXT)

class Kitti {
public:
	Kitti() {
		calib = new CalibData();
	}
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
		cv::Mat k03;
	};

	CalibData *calib;

	void readCalib(std::string filename);
	void readDepth(std::string filename, cv::Mat &depth, cv::Mat &mask);
	void readDepthHalf(std::string filename, cv::Mat &depth, cv::Mat &mask);
	std::vector<float> depthError(cv::Mat depthOut, cv::Mat depthGt, cv::Mat depthMaskGt, cv::Mat &error);
	void useStereoPose0203(cv::Mat &R, cv::Mat &t);
	void errorToColor(cv::Mat error, cv::Mat mask, cv::Mat &outputColor);
};
