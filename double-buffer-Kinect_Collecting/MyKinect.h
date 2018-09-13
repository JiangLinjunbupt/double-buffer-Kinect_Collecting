#pragma once
#include <Kinect.h>
#include"Camera.h"
#include"DataFrame.h"
#include"HandFinder.h"
#include"PointCloud.h"

#include<string>
#include <opencv2\opencv.hpp>
using namespace cv;
using namespace std;


// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


class myKinect
{
private:
	const Camera        * camera;

	IKinectSensor       * mySensor;
	IColorFrameReader   * mycolorReader;
	IDepthFrameReader   * mydepthReader;
	ICoordinateMapper   * myMapper;

	ColorSpacePoint     * m_pcolorcoordinate;

	cv::Mat depth_image[2];
	cv::Mat color_image[2];
	cv::Mat sensor_silhouette_buffer;
	
	vector<Vector3> pointcloud_vector[2];
	Vector3 pointcloud_center[2];

public:
	HandFinder * handfinder;
	PointCloud * pointcloud;
public:
	myKinect(Camera *_camera);
	~myKinect();

	HRESULT  InitializeDefaultSensor();//用于初始化kinect
	void fetch_data(DataFrame &frame, HandFinder & handfinder,PointCloud &other_pointcloud);
	bool run();
};
