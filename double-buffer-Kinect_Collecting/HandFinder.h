#pragma once
#include"Camera.h"
#include "opencv2/core/core.hpp"       ///< cv::Mat
#include "opencv2/highgui/highgui.hpp" ///< cv::imShow
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"   ///< CV_BGRA2RGBA
using namespace cv;

class HandFinder {
private:
	Camera*const camera = NULL;
public:
	HandFinder(Camera * camera);
	~HandFinder() {
		delete[] sensor_indicator;
	}

	/// @{ Settings
public:
	struct Settings {
		bool show_hand = false;
		bool show_wband = false;
		float depth_range = 150;
		float wband_size = 30;
		//这里的颜色分割我没有从文件中读取，直接在下面写死了。红色的HSV主要在于H，H范围（0，8）U（120，180），其他的两个设为合适值即可。
		cv::Scalar hsv_min1 = cv::Scalar(0, 80, 20); ///< potentially read from file
		cv::Scalar hsv_max1 = cv::Scalar(8, 255, 255); ///< potentially read from file

		cv::Scalar hsv_min2 = cv::Scalar(120, 80, 20); ///< potentially read from file
		cv::Scalar hsv_max2 = cv::Scalar(180, 255, 255); ///< potentially read from file
	} _settings;

	Settings*const settings = &_settings;
	/// @}

public:
	bool _has_useful_data = false;
	bool _wristband_found;

public:
	cv::Mat sensor_hand_silhouette; ///< created by binary_classifier             
	cv::Mat mask_wristband; ///< created by binary_classifier, not used anywhere else
	cv::Mat mask_wristband2;
	int * sensor_indicator;
	int num_sensor_points;

public:
	bool has_useful_data() { return _has_useful_data; }
	bool wristband_found() { return _wristband_found; }
public:
	void binary_classification(cv::Mat& depth, cv::Mat& color);
};
