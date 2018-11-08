#include "Kitti.h"

std::vector<float> Kitti::depthError(cv::Mat depthOut, cv::Mat depthGt, cv::Mat depthMaskGt, cv::Mat &error) {
	std::vector<float> errors(9, 0.f);
	// check file size
	if (depthGt.cols != depthOut.cols || depthGt.rows != depthOut.rows) {
		std::cout << "ERROR: Wrong file size!" << std::endl;
		throw 1;
	}
	int width = depthGt.cols;
	int height = depthGt.rows;

	int num_pixels = 0;
	int num_pixels_result = 0;

	//log sum for scale invariant metric
	float logSum = 0.0;

	// for all pixels do
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			if (depthMaskGt.at<float>(j, i) == 1.0f) {
				const float depth_ipol_m = depthOut.at<float>(j, i);
				const float depth_gt_m = depthGt.at<float>(j, i);

				//error if gt is valid
				const float d_err = fabs(depth_gt_m - depth_ipol_m);
				error.at<float>(j, i) = d_err;

				const float d_err_squared = d_err * d_err;
				const float d_err_inv = fabs(1.0f / depth_gt_m - 1.0f / depth_ipol_m);
				const float d_err_inv_squared = d_err_inv * d_err_inv;
				const float d_err_log = fabs(log(depth_gt_m) - log(depth_ipol_m));
				const float d_err_log_squared = d_err_log * d_err_log;

				//mae
				errors[0] += d_err;
				//rmse
				errors[1] += d_err_squared;
				//inv_mae
				errors[2] += d_err_inv;
				//inv_rmse
				errors[3] += d_err_inv_squared;
				//log
				errors[4] += d_err_log;
				//log rmse
				errors[5] += d_err_log_squared;
				//log diff for scale invariant metric
				logSum += (log(depth_gt_m) - log(depth_ipol_m));
				//abs relative
				errors[7] += d_err / depth_gt_m;
				//squared relative
				errors[8] += d_err_squared / (depth_gt_m*depth_gt_m);

				//increase valid gt pixels
				num_pixels++;
			}
		}
	}

	//normalize mae
	errors[0] /= (float)num_pixels;
	//normalize and take sqrt for rmse
	errors[1] /= (float)num_pixels;
	errors[1] = sqrt(errors[1]);
	//normalize inverse absoulte error
	errors[2] /= (float)num_pixels;
	//normalize and take sqrt for inverse rmse
	errors[3] /= (float)num_pixels;
	errors[3] = sqrt(errors[3]);
	//normalize log mae
	errors[4] /= (float)num_pixels;
	//first normalize log rmse -> we need this result later
	const float normalizedSquaredLog = errors[5] / (float)num_pixels;
	errors[5] = sqrt(normalizedSquaredLog);
	//calculate scale invariant metric
	errors[6] = sqrt(normalizedSquaredLog - (logSum*logSum / ((float)num_pixels*(float)num_pixels)));
	//normalize abs relative
	errors[7] /= (float)num_pixels;
	//normalize squared relative
	errors[8] /= (float)num_pixels;

	return errors;
}

void Kitti::readCalib(std::string filename) {
	//read from file
	std::fstream calibFile(filename, std::ios_base::in);
	// Read calibTime;

	std::string h;
	calibFile >> h >> calib->date >> calib->time;
	calibFile >> h >> calib->cornerDist;
	for (int k = 0; k < 4; k++) {
		std::string header;
		double a, b, c, d, e, f, g, h, i;

		calibFile >> header >> a >> b; //s
		double smat[2] = { a, b };
		calib->s[k] = cv::Mat(1, 2, CV_64F, smat).clone();

		calibFile >> header >> a >> b >> c >> d >> e >> f >> g >> h >> i; //k
		double kmat[9] = { a, b, c, d, e, f, g, h, i };
		calib->k[k] = cv::Mat(3, 3, CV_64F, kmat).clone();

		calibFile >> header >> a >> b >> c >> d >> e; //d
		double dmat[5] = { a, b, c, d, e };
		calib->d[k] = cv::Mat(1, 5, CV_64F, dmat).clone();

		calibFile >> header >> a >> b >> c >> d >> e >> f >> g >> h >> i; //r
		double rmat[9] = { a, b, c, d, e, f, g, h, i };
		calib->r[k] = cv::Mat(3, 3, CV_64F, rmat).clone();

		calibFile >> header >> a >> b >> c; //t
		double tmat[5] = { a, b, c, d, e };
		calib->t[k] = cv::Mat(3, 1, CV_64F, tmat).clone();

		calibFile >> header >> a >> b; //srect
		double srmat[2] = { a, b };
		calib->sRect[k] = cv::Mat(1, 2, CV_64F, srmat).clone();

		calibFile >> header >> a >> b >> c >> d >> e >> f >> g >> h >> i; //rrect
		double rrmat[9] = { a, b, c, d, e, f, g, h, i };
		calib->rRect[k] = cv::Mat(3, 3, CV_64F, rrmat).clone();

		double m, n, o;
		calibFile >> header >> a >> b >> c >> d >> e >> f >> g >> h >> i >> m >> n >> o; //prect
		double prmat[12] = { a, b, c, d, e, f, g, h, i, m, n, o };
		calib->pRect[k] = cv::Mat(3, 4, CV_64F, prmat).clone();
	}
	double k02mat[9] = { calib->pRect[2].at<double>(0,0), calib->pRect[2].at<double>(0,1), calib->pRect[2].at<double>(0,2),
		calib->pRect[2].at<double>(1,0), calib->pRect[2].at<double>(1,1), calib->pRect[2].at<double>(1,2),
		calib->pRect[2].at<double>(2,0), calib->pRect[2].at<double>(2,1), calib->pRect[2].at<double>(2,2)
	};
	calib->k02 = cv::Mat(3, 3, CV_64F, k02mat).clone();

	std::cout << "K0: " << calib->k02 << std::endl;

	double k03mat[9] = { calib->pRect[3].at<double>(0,0), calib->pRect[3].at<double>(0,1), calib->pRect[3].at<double>(0,2),
		calib->pRect[3].at<double>(1,0), calib->pRect[3].at<double>(1,1), calib->pRect[3].at<double>(1,2),
		calib->pRect[3].at<double>(2,0), calib->pRect[3].at<double>(2,1), calib->pRect[3].at<double>(2,2)
	};
	calib->k03 = cv::Mat(3, 3, CV_64F, k03mat).clone();

	std::cout << "K1: " << calib->k03 << std::endl;
	//std::cout << calib->k02.at<double>(0, 0) << " " << calib->k02.at<double>(0, 1) << " " << calib->k02.at<double>(0, 2) << std::endl;
	//std::cout << calib->k02.at<double>(1, 0) << " " << calib->k02.at<double>(1, 1) << " " << calib->k02.at<double>(1, 2) << std::endl;
	//std::cout << calib->k02.at<double>(2, 0) << " " << calib->k02.at<double>(2, 1) << " " << calib->k02.at<double>(2, 2) << std::endl;
}

void Kitti::readDepth(std::string filename, cv::Mat &depth, cv::Mat &mask) {
	std::cout << "Reading depth... " << std::endl;
	cv::Mat depthRaw = cv::imread(filename, cv::IMREAD_UNCHANGED);
	depthRaw.convertTo(depth, CV_32F, 1 / 256.0);
	//std::cout << depthRaw.at<unsigned short>(0, 0) << std::endl;
	//std::cout << depth.at<float>(0, 0) << std::endl;

	// Extract the mask
	mask = cv::Mat::zeros(depthRaw.size(), CV_32F);
	for (int j = 0; j < mask.rows; j++) {
		for (int i = 0; i < mask.cols; i++) {
			if (depth.at<float>(j, i) > 0.0f) {
				mask.at<float>(j, i) = 1.0f;
			}
		}
	}
	//cv::imshow("mask", mask);
	//cv::waitKey();
}

void Kitti::useStereoPose0203(cv::Mat &R, cv::Mat &t) {
	//cv::Mat R02 = calib->rRect[2];
	//cv::Mat R03 = calib->rRect[3];
	R = cv::Mat::eye(3, 3, CV_64F);
	double baseline30 = -calib->pRect[3].at<double>(0, 3) / calib->pRect[3].at<double>(0, 0); //-44/721
	double baseline20 = -calib->pRect[2].at<double>(0, 3) / calib->pRect[2].at<double>(0, 0); //+339/721
	double baseline32 = baseline30 - baseline20; // +383/721
	double tvec[3] = { -baseline32, 0.000001, 0.000001 };
	t = cv::Mat(3, 1, CV_64F, tvec).clone();
	std::cout << "Rstereo: " << R << std::endl;
	std::cout << "tstereo: " << t << std::endl;
}

void Kitti::errorToColor(cv::Mat error, cv::Mat mask, cv::Mat &outputColor) {
	int height = mask.rows;
	int width = mask.cols;

	outputColor = cv::Mat::zeros(mask.size(), CV_8UC3);
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			if (mask.at<float>(j, i) == 1.0f) {
				if ((error.at<float>(j, i) >= 0.0f) && (error.at<float>(j, i) < 0.1875f))
					outputColor.at<cv::Vec3b>(j, i) = cv::Vec3b(149, 54, 49);
				else if ((error.at<float>(j, i) >= 0.1875f) && (error.at<float>(j, i) < 0.375f))
					outputColor.at<cv::Vec3b>(j, i) = cv::Vec3b(180, 117, 69);
				else if ((error.at<float>(j, i) >= 0.375f) && (error.at<float>(j, i) < 0.75f))
					outputColor.at<cv::Vec3b>(j, i) = cv::Vec3b(209, 173, 116);
				else if ((error.at<float>(j, i) >= 0.75f) && (error.at<float>(j, i) < 1.5f))
					outputColor.at<cv::Vec3b>(j, i) = cv::Vec3b(233, 217, 171);
				else if ((error.at<float>(j, i) >= 1.5f) && (error.at<float>(j, i) < 3.0f))
					outputColor.at<cv::Vec3b>(j, i) = cv::Vec3b(248, 243, 224);
				else if ((error.at<float>(j, i) >= 3.0f) && (error.at<float>(j, i) < 6.0f))
					outputColor.at<cv::Vec3b>(j, i) = cv::Vec3b(144, 224, 254);
				else if ((error.at<float>(j, i) >= 6.0f) && (error.at<float>(j, i) < 12.0f))
					outputColor.at<cv::Vec3b>(j, i) = cv::Vec3b(97, 174, 253);
				else if ((error.at<float>(j, i) >= 12.0f) && (error.at<float>(j, i) < 24.0f))
					outputColor.at<cv::Vec3b>(j, i) = cv::Vec3b(67, 109, 244);
				else if ((error.at<float>(j, i) >= 24.0f) && (error.at<float>(j, i) < 48.0f))
					outputColor.at<cv::Vec3b>(j, i) = cv::Vec3b(39, 48, 215);
				else if ((error.at<float>(j, i) >= 48.0f))
					outputColor.at<cv::Vec3b>(j, i) = cv::Vec3b(38, 0, 165);
			}
		}
	}

	int dilation_type = cv::MORPH_RECT;
	int dilation_size = 1;

	cv::Mat element = cv::getStructuringElement(dilation_type,
		cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		cv::Point(dilation_size, dilation_size));
	/// Apply the dilation operation
	cv::dilate(outputColor, outputColor, element);
	//imshow("Dilation Demo", dilation_dst);
}