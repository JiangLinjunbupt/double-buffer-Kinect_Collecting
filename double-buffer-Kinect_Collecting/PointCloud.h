#pragma once
#include<opencv2/opencv.hpp>    
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<vector>
#include"Camera.h"
#include"HandFinder.h"
using namespace std;

struct PointCloud
{
	Camera * camera;

	vector<Vector3> pointcloud_vector;

	float PointCloud_center_x, PointCloud_center_y, PointCloud_center_z = 0.0f;

	PointCloud() {}
	~PointCloud() {}

	void DepthMatToPointCloud(cv::Mat& depth, HandFinder* hanfinder)
	{
		pointcloud_vector.clear();

		for (int i = 0; i < hanfinder->num_sensor_points; i++)
		{
			int col = hanfinder->sensor_indicator[i] % 512;        //x
			int row = hanfinder->sensor_indicator[i] / 512;        //y

			Integer z = depth.at<unsigned short>(row, col);

			Vector3 p_pixel = camera->depth_to_world(row, col, z);

			p_pixel.z() = p_pixel.z();    //这里加负号的原因：我设置的虚拟摄像机是看向Z轴负方向，而现实中的摄像机是看向Z轴正方向的
			this->PointCloud_center_x += p_pixel.x();
			this->PointCloud_center_y += p_pixel.y();
			this->PointCloud_center_z += p_pixel.z();

			this->pointcloud_vector.push_back(p_pixel);

		}

		if (this->pointcloud_vector.size() != 0)
		{
			this->PointCloud_center_x = this->PointCloud_center_x / (float)(this->pointcloud_vector.size());
			this->PointCloud_center_y = this->PointCloud_center_y / (float)(this->pointcloud_vector.size());
			this->PointCloud_center_z = this->PointCloud_center_z / (float)(this->pointcloud_vector.size());
		}
	}

	void DepthMatToPointCloud(cv::Mat& depth, int *indicator,int NUM_indicator)
	{
		pointcloud_vector.clear();

		for (int i = 0; i < NUM_indicator; i++)
		{
			int index = indicator[i];
			int col = index % 512;        //x
			int row = index / 512;        //y

			Integer z = depth.at<unsigned short>(row, col);

			Vector3 p_pixel = camera->depth_to_world(row, col, z);

			p_pixel.z() = p_pixel.z();    //这里加负号的原因：我设置的虚拟摄像机是看向Z轴负方向，而现实中的摄像机是看向Z轴正方向的
			this->PointCloud_center_x += p_pixel.x();
			this->PointCloud_center_y += p_pixel.y();
			this->PointCloud_center_z += p_pixel.z();

			this->pointcloud_vector.push_back(p_pixel);

		}

		if (this->pointcloud_vector.size() != 0)
		{
			this->PointCloud_center_x = this->PointCloud_center_x / (float)(this->pointcloud_vector.size());
			this->PointCloud_center_y = this->PointCloud_center_y / (float)(this->pointcloud_vector.size());
			this->PointCloud_center_z = this->PointCloud_center_z / (float)(this->pointcloud_vector.size());
		}
	}

};