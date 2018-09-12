#include"HandFinder.h"
#include <numeric> ///< std::iota

#define CHECK_NOTNULL(val) if(val==NULL){ std::cout << "!!!CHECK_NOT_NULL: " << __FILE__ << " " << __LINE__ << std::endl; exit(0); }

HandFinder::HandFinder(Camera *camera) :camera(camera) {
	CHECK_NOTNULL(camera);
	sensor_indicator = new int[424 * 512];

	sensor_hand_silhouette = Mat::zeros(424, 512, CV_8UC1);
}

Vector3 point_at_depth_pixel(cv::Mat& depth, int x, int y, Camera* camera) {
	Integer z = depth.at<unsigned short>(y, x);   //这里是（y,x)的原因：Mat中的.at跟的是第几行，第几列，对应着就是第y行，第x列
	return camera->depth_to_world(x, y, z);
}

void HandFinder::binary_classification(cv::Mat& depth, cv::Mat& color) {
	_wristband_found = false;


	float wband_size = _settings.wband_size;  //30
	float depth_range = _settings.depth_range;  //150

	///--- We look for wristband up to here...

	///--- Allocated once
	static cv::Mat color_hsv;
	static cv::Mat in_z_range;
	static cv::Mat depth_copy;

	// TIMED_BLOCK(timer,"Worker_classify::(convert to HSV)")
	{
		cv::cvtColor(color, color_hsv, CV_BGR2HSV);     //rgb在opencv中存储为BGR的顺序，因此，这里应该为BGR2HSV而不是RGB2HSV

		cv::inRange(color_hsv, settings->hsv_min1, settings->hsv_max1, /*=*/ mask_wristband);
		cv::inRange(color_hsv, settings->hsv_min2, settings->hsv_max2, /*=*/ mask_wristband2);
		cv::bitwise_or(mask_wristband, mask_wristband2, mask_wristband);

		depth_copy = depth.clone();
		// 首先处理深度为 0 的点，这种实际上无法测量的点，
		//所以将深度为 0 的点设成最大深度
		for (int i = 0; i < 424; i++)
		{
			for (int j = 0; j < 512; j++)
			{
				//unsigned short temp = depthData[i * 512 + j];
				unsigned short temp = depth_copy.at<ushort>(i, j);
				if (temp == 0)
				{
					//temp = 65535;//16位最大值为65535
					depth_copy.at<ushort>(i, j) = 65535;
				}
			}
		}

		GaussianBlur(depth_copy, depth_copy, Size(9, 9), 0.85, 0.85);

		double minValue, maxValue;
		minMaxIdx(depth_copy, &minValue, &maxValue);

		float threshold = 120;//单位mm
		cv::inRange(depth, minValue, minValue + threshold /*mm*/, /*=*/ in_z_range);

		cv::bitwise_and(mask_wristband, in_z_range, mask_wristband);

		//cv::imshow("mask_wristband (pre)", mask_wristband); cv::waitKey(1);
	}

	//imshow("red part", mask_wristband);

	// TIMED_BLOCK(timer,"Worker_classify::(robust wrist)")
	{
		cv::Mat labels, stats, centroids;
		int num_components = cv::connectedComponentsWithStats(mask_wristband, labels, stats, centroids, 4 /*connectivity={4,8}*/);

		///--- Generate array to sort
		std::vector< int > to_sort(num_components);
		std::iota(to_sort.begin(), to_sort.end(), 0 /*start from*/);

		///--- Sort accoding to area
		auto lambda = [stats](int i1, int i2) {
			int area1 = stats.at<int>(i1, cv::CC_STAT_AREA);
			int area2 = stats.at<int>(i2, cv::CC_STAT_AREA);
			return area1>area2;
		};
		std::sort(to_sort.begin(), to_sort.end(), lambda);

		if (num_components<2 /*not found anything beyond background*/) {
			cout << "wrist band not find !!! " << endl;
			_has_useful_data = false;
		}
		else
		{
			_has_useful_data = true;

			///--- Select 2nd biggest component  //因为面积最大的是背景
			mask_wristband = (labels == to_sort[1]);
			_wristband_found = true;
		}
	}


	if (_has_useful_data)
	{
		if (_settings.show_wband) {
			cv::imshow("show_wband", mask_wristband);
			cv::waitKey(1);
		}
		else
			cv::destroyWindow("show_wband");


		cv::bitwise_xor(in_z_range, mask_wristband, sensor_hand_silhouette);


		cv::Mat labels, stats, centroids;
		int num_components = cv::connectedComponentsWithStats(sensor_hand_silhouette, labels, stats, centroids, 4 /*connectivity={4,8}*/);

		///--- Generate array to sort
		std::vector< int > to_sort(num_components);
		std::iota(to_sort.begin(), to_sort.end(), 0 /*start from*/);

		///--- Sort accoding to area
		auto lambda = [stats](int i1, int i2) {
			int area1 = stats.at<int>(i1, cv::CC_STAT_AREA);
			int area2 = stats.at<int>(i2, cv::CC_STAT_AREA);
			return area1>area2;
		};
		std::sort(to_sort.begin(), to_sort.end(), lambda);

		if (num_components<2 /*not found anything beyond background*/) {
			cout << "hand not find !!! " << endl;
		}
		else
		{
			///--- Select 2nd biggest component  //因为面积最大的是背景
			sensor_hand_silhouette = (labels == to_sort[1]);
		}

		GaussianBlur(sensor_hand_silhouette, sensor_hand_silhouette, Size(5, 5), 0.85, 0.85);

		if (_settings.show_hand) {
			cv::imshow("show_hand", sensor_hand_silhouette);
		}
		else {
			cv::destroyWindow("show_hand");
		}
	}

}