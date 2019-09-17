#include "Ivdata.h"

int Ivdata::readIntrinsic(std::string filename, cv::Mat &K) {
	std::fstream calibFile(filename, std::ios_base::in);
	double a, b, c, d, e, f, g, h, i;
	calibFile >> a >> b >> c >> d >> e >> f >> g >> h >> i;
	double k0[9] = { a, b, c, d, e, f, g, h, i };
	K = cv::Mat(3, 3, CV_64F, k0).clone();
	return 0;
}

int Ivdata::readMLP(std::string filename, cv::Mat &R, cv::Mat &t) {
	std::fstream calibFile(filename, std::ios_base::in);
	std::string header;
	for (int z = 0; z < 5; z++) {
		std::getline(calibFile, header); // doctype
		//std::cout << header << std::endl;
	}

	double a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p;
	calibFile >> a >> b >> c >> d >> e >> f >> g >> h >> i >> j >> k >> l >> m >> n >> o >> p;
	double p0[16] = { a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p};
	cv::Mat P0 = cv::Mat(4, 4, CV_64F, p0).clone();
	//std::cout << "P0: " << P0 << std::endl;

	for (int z = 0; z < 5; z++) {
		std::getline(calibFile, header); // doctype
		//std::cout << header << std::endl;
	}

	calibFile >> a >> b >> c >> d >> e >> f >> g >> h >> i >> j >> k >> l >> m >> n >> o >> p;
	double p1[16] = { a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p };

	cv::Mat P1 = cv::Mat(4, 4, CV_64F, p1).clone();
	//std::cout << "P1: " << P1 << std::endl;

	double r[9] = { a, b, c, e, f, g, i, j, k };
	R = cv::Mat(3, 3, CV_64F, r).clone();

	double tt[3] = { d, h, l };
	t = cv::Mat(3, 1, CV_64F, tt).clone();

	std::cout << "R: " << R << std::endl;
	std::cout << "t: " << t << std::endl;

	return 0;
}

int Ivdata::readDepth(std::string filename, cv::Mat &depth, cv::Mat &mask) {
	std::cout << "Reading depth... " << std::endl;
	cv::Mat depthRaw = cv::imread(filename, cv::IMREAD_UNCHANGED);
	depthRaw.convertTo(depth, CV_32F, 1 / 256.0);
	//std::cout << depthRaw.at<unsigned short>(0, 0) << std::endl;
	//std::cout << depth.at<float>(0, 0) << std::endl;

	// Extract the mask				
	mask = cv::Mat::zeros(depthRaw.size(), CV_32F);
	for (int j = 0; j < mask.rows; j++) {
		for (int i = 0; i < mask.cols; i++) {
			if (depth.at<float>(j, i) > 0.5f) {
				mask.at<float>(j, i) = 1.0f;
			}
		}
	}
	return 0;
}