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
	//kinect 2.0 的深度空间的高*宽是 424 * 512，在官网上有说明
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;

private:
	const Camera        * camera;

	IKinectSensor       * mySensor;

	IColorFrameReader   * mycolorReader;
	IDepthFrameReader   * mydepthReader;
	IBodyFrameReader	* myBodyReader;
	IBodyFrameSource	* myBodySource;
	ICoordinateMapper   * myMapper;

	ColorSpacePoint     * m_pcolorcoordinate;
	CameraSpacePoint    * m_pcameracoordinate;

	cv::Mat depth_image[2];
	cv::Mat color_image[2];
	cv::Mat sensor_silhouette_buffer;
	
	vector<Vector3> pointcloud_vector[2];
	Vector3 pointcloud_center[2];

	Mat m_middepth8u;

	int indicator[cDepthWidth*cDepthHeight];
	int NUM_indicator ;

	int indicator_2[cDepthWidth*cDepthHeight];
	int NUM_indicator_2;

public:
	HandFinder * handfinder;
	PointCloud * pointcloud;
public:
	myKinect(Camera *_camera);
	~myKinect();

	HRESULT  InitializeDefaultSensor();//用于初始化kinect
	void fetch_data(DataFrame &frame, HandFinder & handfinder,PointCloud &other_pointcloud);

	void getData();
	bool run();

	void run2();
};
