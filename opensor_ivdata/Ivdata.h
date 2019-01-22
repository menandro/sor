#pragma once
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

class Ivdata {
public:
	Ivdata() {};
	~Ivdata() {};

	int readIntrinsic(std::string filename, cv::Mat& K);
	int readMLP(std::string filename, cv::Mat& R, cv::Mat& t);
	int readDepth(std::string filename, cv::Mat &depth, cv::Mat &mask);
};