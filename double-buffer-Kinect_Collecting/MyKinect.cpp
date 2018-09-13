#include "myKinect.h"
#include <math.h>
#include <iostream>
#include <stdio.h>

#include <thread>
#include <mutex>
#include <condition_variable>

std::thread sensor_thread;
std::mutex swap_mutex;
std::condition_variable condition;
bool main_released = true;
bool thread_released = false;

bool wristband_found_buffer;

const int BACK_BUFFER = 1;
const int FRONT_BUFFER = 0;


myKinect::myKinect(Camera *_camera) :
	camera(_camera),
	mySensor(NULL),
	mydepthReader(NULL),
	mycolorReader(NULL),
	myMapper(NULL)
{
	this->handfinder = new HandFinder(_camera);
	this->pointcloud = new PointCloud();
	this->pointcloud->camera = _camera;


	int cDepthHeight = camera->height();
	int cDepthWidth = camera->width();

	depth_image[FRONT_BUFFER] = Mat::zeros(cDepthHeight, cDepthWidth, CV_16UC1);
	depth_image[BACK_BUFFER] = Mat::zeros(cDepthHeight, cDepthWidth, CV_16UC1);

	color_image[FRONT_BUFFER] = Mat(cDepthHeight, cDepthWidth, CV_8UC3, cv::Scalar(0, 0, 0));
	color_image[BACK_BUFFER] = Mat(cDepthHeight, cDepthWidth, CV_8UC3, cv::Scalar(0, 0, 0));

	pointcloud_vector[FRONT_BUFFER].clear();
	pointcloud_vector[BACK_BUFFER].clear();

	sensor_silhouette_buffer = Mat::zeros(cDepthHeight, cDepthWidth, CV_8UC1);

	this->m_pcolorcoordinate = new ColorSpacePoint[512 * 424];
}


myKinect::~myKinect()
{
	SafeRelease(myMapper);
	SafeRelease(mycolorReader);
	SafeRelease(mydepthReader);

	if (mySensor)
	{
		mySensor->Close();
	}

	SafeRelease(mySensor);
}

HRESULT myKinect::InitializeDefaultSensor()
{
	HRESULT hr;
	//搜索kinect
	hr = GetDefaultKinectSensor(&mySensor);
	if (FAILED(hr)) {
		return hr;
	}
	if (mySensor)
	{
		// Initialize the Kinect and get coordinate mapper and the body reader
		IColorFrameSource   * mycolorSource = nullptr;
		IDepthFrameSource   * mydepthSource = nullptr;   //取得深度数据

		hr = mySensor->Open();    //打开kinect 

								  //coordinatemapper
		if (SUCCEEDED(hr))
		{
			hr = mySensor->get_CoordinateMapper(&myMapper);
		}
		//color
		if (SUCCEEDED(hr))
		{
			hr = mySensor->get_ColorFrameSource(&mycolorSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = mycolorSource->OpenReader(&mycolorReader);
		}

		//depth
		if (SUCCEEDED(hr)) {
			hr = mySensor->get_DepthFrameSource(&mydepthSource);
		}

		if (SUCCEEDED(hr)) {
			hr = mydepthSource->OpenReader(&mydepthReader);
		}

		SafeRelease(mycolorSource);
		SafeRelease(mydepthSource);
	}

	if (!mySensor || FAILED(hr))
	{
		std::cerr << "Kinect initialization failed!" << std::endl;
		return E_FAIL;
	}


	sensor_thread = std::thread(&myKinect::run, this);
	sensor_thread.detach();

	return hr;

}

void myKinect::fetch_data(DataFrame &frame, HandFinder & other_handfinder,PointCloud &other_pointcloud)
{
	std::unique_lock<std::mutex> lock(swap_mutex);
	condition.wait(lock, [] {return thread_released; });
	main_released = false;

	//frame.color = color_image[FRONT_BUFFER].clone();
	frame.depth = depth_image[FRONT_BUFFER].clone();

	other_handfinder.sensor_hand_silhouette = sensor_silhouette_buffer.clone();
	other_handfinder._wristband_found = wristband_found_buffer;

	//other_pointcloud.pointcloud_vector.swap(pointcloud_vector[FRONT_BUFFER]);
	other_pointcloud.pointcloud_vector.assign(pointcloud_vector[FRONT_BUFFER].begin(), pointcloud_vector[FRONT_BUFFER].end());

	other_pointcloud.PointCloud_center_x = pointcloud_center[FRONT_BUFFER].x();
	other_pointcloud.PointCloud_center_y = pointcloud_center[FRONT_BUFFER].y();
	other_pointcloud.PointCloud_center_z = pointcloud_center[FRONT_BUFFER].z();

	main_released = true;
	lock.unlock();
	condition.notify_one();
}

bool myKinect::run()
{
	cout << "Kinect.run()" << endl;
	Mat image_color = Mat::zeros(1080, 1920, CV_8UC4);
	UINT16 *depthData = new UINT16[424 * 512];
	for (;;)
	{
		//如果丢失了kinect，则不继续操作
		if (!mydepthReader)
		{
			std::cout << "the depth reader is Missing, reboot the Kinect!!" << std::endl;
			return false;
		}

		IDepthFrame     * mydepthFrame = nullptr;
		IColorFrame     * mycolorFrame = nullptr;

		if (mydepthReader->AcquireLatestFrame(&mydepthFrame) == S_OK)
		{
			mydepthFrame->CopyFrameDataToArray(424 * 512, (UINT16 *)depth_image[BACK_BUFFER].data);
			//imshow("original depth", original_depth_16U);
			mydepthFrame->CopyFrameDataToArray(424 * 512, depthData); //先把数据存入16位的图像矩阵中
		}

		if (mycolorReader->AcquireLatestFrame(&mycolorFrame) == S_OK)
		{
			mycolorFrame->CopyConvertedFrameDataToArray(1080 * 1920 * 4, (BYTE *)image_color.data, ColorImageFormat_Bgra);
			while (myMapper->MapDepthFrameToColorSpace(424 * 512, depthData, 424 * 512, m_pcolorcoordinate) != S_OK) { ; }

			for (int i = 0; i < 424; i++)
			{
				for (int j = 0; j < 512; j++)
				{
					int index_depth = i * 512 + j;
					ColorSpacePoint pp = m_pcolorcoordinate[index_depth];
					if (pp.X != -std::numeric_limits<float>::infinity() && pp.Y != -std::numeric_limits<float>::infinity())
					{
						int colorX = static_cast<int>(pp.X + 0.5f);
						int colorY = static_cast<int>(pp.Y + 0.5f);
						if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
						{
							unsigned char b = image_color.at<cv::Vec4b>(colorY, colorX)[0];
							unsigned char g = image_color.at<cv::Vec4b>(colorY, colorX)[1];
							unsigned char r = image_color.at<cv::Vec4b>(colorY, colorX)[2];
							color_image[BACK_BUFFER].at<cv::Vec3b>(i, j) = cv::Vec3b(b, g, r);
						}
					}
				}
			}

		}


		SafeRelease(mydepthFrame);
		SafeRelease(mycolorFrame);

		handfinder->binary_classification(depth_image[BACK_BUFFER], color_image[BACK_BUFFER]);
		handfinder->num_sensor_points = 0;

		int count = 0;
		for (int row = 0; row < handfinder->sensor_hand_silhouette.rows; ++row) {
			for (int col = 0; col < handfinder->sensor_hand_silhouette.cols; ++col) {
				if (handfinder->sensor_hand_silhouette.at<uchar>(row, col) != 255) continue;
				if (count % 2 == 0) {
					handfinder->sensor_indicator[handfinder->num_sensor_points] = row * handfinder->sensor_hand_silhouette.cols + col;
					handfinder->num_sensor_points++;
				}
				count++;
			}
		}



		//cout << "handfinder done!" << endl;

		pointcloud->DepthMatToPointCloud(depth_image[BACK_BUFFER], handfinder);


		pointcloud_vector[BACK_BUFFER].swap(pointcloud->pointcloud_vector);
		pointcloud_center[BACK_BUFFER] << pointcloud->PointCloud_center_x, pointcloud->PointCloud_center_y, pointcloud->PointCloud_center_z;


		// Lock the mutex and swap the buffers
		{
			std::unique_lock<std::mutex> lock(swap_mutex);
			condition.wait(lock, [] {return main_released; });
			thread_released = false;

			depth_image[FRONT_BUFFER] = depth_image[BACK_BUFFER].clone();
			//color_image[FRONT_BUFFER] = color_image[BACK_BUFFER].clone();


			sensor_silhouette_buffer = handfinder->sensor_hand_silhouette.clone();
			wristband_found_buffer = handfinder->_wristband_found;

			pointcloud_vector[FRONT_BUFFER].swap(pointcloud_vector[BACK_BUFFER]);
			pointcloud_center[FRONT_BUFFER] << pointcloud_center[BACK_BUFFER];

			thread_released = true;
			lock.unlock();
			condition.notify_one();
		}
	}

	cout << "for no reason the run() breakdown!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;

}