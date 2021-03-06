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
	myBodyReader(NULL),
	myBodySource(NULL),
	myMapper(NULL)
{
	this->handfinder = new HandFinder(_camera);
	this->pointcloud = new PointCloud();
	this->pointcloud->camera = _camera;

	depth_image[FRONT_BUFFER] = Mat::zeros(cDepthHeight, cDepthWidth, CV_16UC1);
	depth_image[BACK_BUFFER] = Mat::zeros(cDepthHeight, cDepthWidth, CV_16UC1);

	color_image[FRONT_BUFFER] = Mat(cDepthHeight, cDepthWidth, CV_8UC3, cv::Scalar(0, 0, 0));
	color_image[BACK_BUFFER] = Mat(cDepthHeight, cDepthWidth, CV_8UC3, cv::Scalar(0, 0, 0));

	pointcloud_vector[FRONT_BUFFER].clear();
	pointcloud_vector[BACK_BUFFER].clear();

	sensor_silhouette_buffer = Mat::zeros(cDepthHeight, cDepthWidth, CV_8UC1);

	this->m_pcolorcoordinate = new ColorSpacePoint[512 * 424];
	this->m_pcameracoordinate = new CameraSpacePoint[512 * 424];


	m_middepth8u = Mat::zeros(cDepthHeight, cDepthWidth, CV_8UC1);
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

		//body
		if (SUCCEEDED(hr)) {
			hr = mySensor->get_BodyFrameSource(&myBodySource);
		}

		if (SUCCEEDED(hr)) {
			hr = myBodySource->OpenReader(&myBodyReader);
		}

		SafeRelease(mycolorSource);
		SafeRelease(mydepthSource);
	}

	if (!mySensor || FAILED(hr))
	{
		std::cerr << "Kinect initialization failed!" << std::endl;
		return E_FAIL;
	}


	sensor_thread = std::thread(&myKinect::run2, this);
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


void myKinect::getData()
{
	//如果丢失了kinect，则不继续操作
	if (!mydepthReader)
	{
		return;
	}

	IBodyFrame		* myBodyFrame = nullptr;
	IDepthFrame     * mydepthFrame = nullptr;
	IColorFrame     * mycolorFrame = nullptr;

	if (mydepthReader->AcquireLatestFrame(&mydepthFrame) == S_OK) {
		mydepthFrame->CopyFrameDataToArray(cDepthHeight * cDepthWidth, (UINT16 *)depth_image[BACK_BUFFER].data); //先把数据存入16位的图像矩阵中																		


		while (myMapper->MapDepthFrameToCameraSpace(424 * 512, (UINT16 *)depth_image[BACK_BUFFER].data, 424 * 512, m_pcameracoordinate) != S_OK) { ; }

		while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK) { ; }			//读取Body数据

		int	bodyBufferSize = 0;
		myBodySource->get_BodyCount(&bodyBufferSize);
		IBody	** bodyArray = new IBody *[bodyBufferSize];
		for (int i = 0; i < bodyBufferSize; i++)
			bodyArray[i] = nullptr;
		myBodyFrame->GetAndRefreshBodyData(bodyBufferSize, bodyArray);

		int nearestBodyindex = -1;
		float MinHead_Z = 100;
		CameraSpacePoint nearestRightHandPosition;
		CameraSpacePoint nearestRightElbowPosition;
		for (int i = 0; i < bodyBufferSize; i++)					//遍历6个人
		{
			BOOLEAN		result = false;
			if (bodyArray[i]->get_IsTracked(&result) == S_OK && result)
			{

				_Joint	jointArray[JointType_Count];				//将关节点输出，正式开始处理
				bodyArray[i]->GetJoints(JointType_Count, jointArray);

				if (jointArray[JointType_Head].TrackingState == TrackingState_Tracked&&jointArray[JointType_HandRight].TrackingState == TrackingState_Tracked)
				{
					if (jointArray[JointType_Head].Position.Z < MinHead_Z)
					{
						MinHead_Z = jointArray[JointType_Head].Position.Z;
						nearestBodyindex = i;
						nearestRightHandPosition = jointArray[JointType_HandRight].Position;
						nearestRightElbowPosition = jointArray[JointType_ElbowRight].Position;
					}
				}
			}


		}

		DepthSpacePoint depthSpaceElbowPosition;
		myMapper->MapCameraPointToDepthSpace(nearestRightElbowPosition, &depthSpaceElbowPosition);


		NUM_indicator = 0;
		for (int i = 0; i < cDepthWidth*cDepthHeight; ++i)
		{
			int row_ = i / cDepthWidth;
			int col_ = i % cDepthWidth;
			CameraSpacePoint p = m_pcameracoordinate[i];
			float distance = sqrt(pow((p.X - nearestRightHandPosition.X), 2)
				+ pow(p.Y - nearestRightHandPosition.Y, 2)
				+ pow(p.Z - nearestRightHandPosition.Z, 2));

			if (distance <0.12)
			{
				indicator[NUM_indicator++] = i;
				m_middepth8u.at<uchar>(row_, col_) = 255;
			}
			else
			{
				m_middepth8u.at<uchar>(row_, col_) = 0;
			}
		}


		if (NUM_indicator != 0)
		{
			Mat data_pts = Mat(NUM_indicator, 2, CV_32FC1);
			for (int i = 0; i < NUM_indicator; ++i)
			{
				data_pts.at<float>(i, 0) = (float)(indicator[i] % 512);
				data_pts.at<float>(i, 1) = (float)(indicator[i] / 512);
			}

			PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
			Point pos = Point(pca_analysis.mean.at<float>(0, 0), pca_analysis.mean.at<float>(0, 1));
			vector<Point2f> eigen_vecs(2);
			vector<float> eigen_val(2);

			eigen_vecs[0] = Point2f(pca_analysis.eigenvectors.at<float>(0, 0), pca_analysis.eigenvectors.at<float>(0, 1));
			eigen_vecs[1] = Point2f(pca_analysis.eigenvectors.at<float>(1, 0), pca_analysis.eigenvectors.at<float>(1, 1));
			eigen_val[0] = pca_analysis.eigenvalues.at<float>(0, 0);
			eigen_val[1] = pca_analysis.eigenvalues.at<float>(1, 0);


			Mat dist_image;
			distanceTransform(m_middepth8u, dist_image, CV_DIST_L2, 3);
			int temp = 0, R = 0, cx = 0, cy = 0;
			for (int i = 0; i < m_middepth8u.rows; i++)
				for (int j = 0; j < m_middepth8u.cols; j++)
				{
					/* checks if the point is inside the contour. Optionally computes the signed distance from the point to the contour boundary*/
					if (m_middepth8u.at<uchar>(i, j) == 255)
					{
						temp = (int)dist_image.ptr<float>(i)[j];
						if (temp > R)
						{
							R = temp;
							cy = i;
							cx = j;
						}

					}
				}


			//判断主方向是否需要反向，根据内切圆和手肘点
			{
				Point2f o = cv::Point2i(cx, cy);
				Vector2f circlr_to_elbow;
				circlr_to_elbow.x() = depthSpaceElbowPosition.X - o.x;
				circlr_to_elbow.y() = depthSpaceElbowPosition.Y - o.y;

				Vector2f main_direction(eigen_vecs[0].x, eigen_vecs[0].y);

				if (main_direction.dot(circlr_to_elbow) < 0)
				{
					eigen_vecs[0].x = -eigen_vecs[0].x;
					eigen_vecs[0].y = -eigen_vecs[0].y;
				}
			}

			for (int i = 0; i < NUM_indicator; ++i)
			{
				int row_ = indicator[i] / 512;  // y
				int col_ = indicator[i] % 512;   //x

				Point2f o = cv::Point2i(cx, cy) + Point(eigen_vecs[0].x*1.2f*R, eigen_vecs[0].y*1.2f*R);
				Vector2f p0(eigen_vecs[0].x, eigen_vecs[0].y);
				Vector2f p;
				p.x() = col_ - o.x; p.y() = row_ - o.y;

				if(p.dot(p0)>0)
				{
					m_middepth8u.at<uchar>(row_, col_) = 0;
				}
			}

			circle(m_middepth8u, Point2i(depthSpaceElbowPosition.X, depthSpaceElbowPosition.Y), 3, Scalar(255, 255, 255), 2);

			circle(m_middepth8u, cv::Point2i(cx, cy), 3, Scalar(0, 0, 0), 2);
			circle(m_middepth8u, cv::Point2i(cx, cy), R, Scalar(0, 0, 0), 1);

			line(m_middepth8u, cv::Point2i(cx, cy), cv::Point2i(cx, cy) + 0.05f*Point(eigen_vecs[0].x*eigen_val[0], eigen_vecs[0].y*eigen_val[0]), Scalar(0, 0, 0));
			line(m_middepth8u, cv::Point2i(cx, cy), cv::Point2i(cx, cy) + 0.05f*Point(eigen_vecs[1].x*eigen_val[1], eigen_vecs[1].y*eigen_val[1]), Scalar(0, 0, 0));


			line(m_middepth8u, cv::Point2i(cx, cy) + Point(eigen_vecs[0].x*1.2f*R, eigen_vecs[0].y*1.2f*R)
				, cv::Point2i(cx, cy) + Point(eigen_vecs[0].x*1.2f*R, eigen_vecs[0].y*1.2f*R) + Point(eigen_vecs[1].x*eigen_val[1], eigen_vecs[1].y*eigen_val[1])
				, Scalar(0, 0, 0));
	
			cout << eigen_vecs[0].x << "            " << eigen_vecs[0].y << endl;
		}
		
		imshow("m_middepth8u", m_middepth8u);
		cvWaitKey(10);
		//delete[] bodyArray;
	}
	SafeRelease(myBodyFrame);
	SafeRelease(mydepthFrame);
	SafeRelease(mycolorFrame);
}


void myKinect::run2()
{
	cout << "Kinect.run()" << endl;
	for (;;)
	{
		//如果丢失了kinect，则不继续操作
		if (!mydepthReader)
		{
			std::cout << "the depth reader is Missing, reboot the Kinect!!" << std::endl;
			return;
		}

		IBodyFrame		* myBodyFrame = nullptr;
		IDepthFrame     * mydepthFrame = nullptr;

		if (mydepthReader->AcquireLatestFrame(&mydepthFrame) == S_OK) {
			mydepthFrame->CopyFrameDataToArray(cDepthHeight * cDepthWidth, (UINT16 *)depth_image[BACK_BUFFER].data); //先把数据存入16位的图像矩阵中																		
		}
		else
		{
			//cout << "can not get depth data" << endl;
			continue;
		}

		SafeRelease(mydepthFrame);
		CameraSpacePoint nearestRightHandPosition;
		CameraSpacePoint nearestRightElbowPosition;
		DepthSpacePoint depthSpaceElbowPosition;

		if (myBodyReader->AcquireLatestFrame(&myBodyFrame) == S_OK)
		{
			int	bodyBufferSize = 0;
			myBodySource->get_BodyCount(&bodyBufferSize);
			IBody	** bodyArray = new IBody *[bodyBufferSize];
			for (int i = 0; i < bodyBufferSize; i++)
				bodyArray[i] = nullptr;
			myBodyFrame->GetAndRefreshBodyData(bodyBufferSize, bodyArray);

			int nearestBodyindex = -1;
			float MinHead_Z = 100;
			for (int i = 0; i < bodyBufferSize; i++)					//遍历6个人
			{
				BOOLEAN		result = false;
				if (bodyArray[i]->get_IsTracked(&result) == S_OK && result)
				{

					_Joint	jointArray[JointType_Count];				//将关节点输出，正式开始处理
					bodyArray[i]->GetJoints(JointType_Count, jointArray);

					if (jointArray[JointType_Head].TrackingState == TrackingState_Tracked&&jointArray[JointType_HandRight].TrackingState == TrackingState_Tracked)
					{
						if (jointArray[JointType_Head].Position.Z < MinHead_Z)
						{
							MinHead_Z = jointArray[JointType_Head].Position.Z;
							nearestBodyindex = i;
							nearestRightHandPosition = jointArray[JointType_HandRight].Position;
							nearestRightElbowPosition = jointArray[JointType_ElbowRight].Position;
						}
					}
				}


			}

			myMapper->MapCameraPointToDepthSpace(nearestRightElbowPosition, &depthSpaceElbowPosition);
		}
		else
		{
			//cout << "can not get body data" << endl;
			continue;
		}


		SafeRelease(myBodyFrame);
		if (myMapper->MapDepthFrameToCameraSpace(424 * 512, (UINT16 *)depth_image[BACK_BUFFER].data, 424 * 512, m_pcameracoordinate) == S_OK)
		{
			NUM_indicator = 0;
			for (int i = 0; i < cDepthWidth*cDepthHeight; ++i)
			{
				int row_ = i / cDepthWidth;
				int col_ = i % cDepthWidth;
				CameraSpacePoint p = m_pcameracoordinate[i];
				float distance = sqrt(pow((p.X - nearestRightHandPosition.X), 2)
					+ pow(p.Y - nearestRightHandPosition.Y, 2)
					+ pow(p.Z - nearestRightHandPosition.Z, 2));

				if (distance <0.12)
				{
					indicator[NUM_indicator++] = i;
					m_middepth8u.at<uchar>(row_, col_) = 255;
				}
				else
				{
					m_middepth8u.at<uchar>(row_, col_) = 0;
				}
			}
		}
		else
		{
			//cout << "can not Map success" << endl;
			continue;
		}


		if (NUM_indicator != 0)
		{
			Mat data_pts = Mat(NUM_indicator, 2, CV_32FC1);
			for (int i = 0; i < NUM_indicator; ++i)
			{
				data_pts.at<float>(i, 0) = (float)(indicator[i] % 512);
				data_pts.at<float>(i, 1) = (float)(indicator[i] / 512);
			}

			PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
			Point pos = Point(pca_analysis.mean.at<float>(0, 0), pca_analysis.mean.at<float>(0, 1));
			vector<Point2f> eigen_vecs(2);
			vector<float> eigen_val(2);

			eigen_vecs[0] = Point2f(pca_analysis.eigenvectors.at<float>(0, 0), pca_analysis.eigenvectors.at<float>(0, 1));
			eigen_vecs[1] = Point2f(pca_analysis.eigenvectors.at<float>(1, 0), pca_analysis.eigenvectors.at<float>(1, 1));
			eigen_val[0] = pca_analysis.eigenvalues.at<float>(0, 0);
			eigen_val[1] = pca_analysis.eigenvalues.at<float>(1, 0);


			Mat dist_image;
			distanceTransform(m_middepth8u, dist_image, CV_DIST_L2, 3);
			int temp = 0, R = 0, cx = 0, cy = 0;
			for (int i = 0; i < m_middepth8u.rows; i++)
				for (int j = 0; j < m_middepth8u.cols; j++)
				{
					/* checks if the point is inside the contour. Optionally computes the signed distance from the point to the contour boundary*/
					if (m_middepth8u.at<uchar>(i, j) == 255)
					{
						temp = (int)dist_image.ptr<float>(i)[j];
						if (temp > R)
						{
							R = temp;
							cy = i;
							cx = j;
						}

					}
				}

			//判断主方向是否需要反向，根据内切圆和手肘点
			{
				Point2f o = cv::Point2i(cx, cy);
				Vector2f circlr_to_elbow;
				circlr_to_elbow.x() = depthSpaceElbowPosition.X - o.x;
				circlr_to_elbow.y() = depthSpaceElbowPosition.Y - o.y;

				Vector2f main_direction(eigen_vecs[0].x, eigen_vecs[0].y);

				if (main_direction.dot(circlr_to_elbow) < 0)
				{
					eigen_vecs[0].x = -eigen_vecs[0].x;
					eigen_vecs[0].y = -eigen_vecs[0].y;
				}
			}


			NUM_indicator_2 = 0;
			for (int i = 0; i < NUM_indicator; ++i)
			{
				int row_ = indicator[i] / 512;  // y
				int col_ = indicator[i] % 512;   //x

				Point2f o = cv::Point2i(cx, cy) + Point(eigen_vecs[0].x*1.2f*R, eigen_vecs[0].y*1.2f*R);
				Vector2f p0(eigen_vecs[0].x, eigen_vecs[0].y);
				Vector2f p;
				p.x() = col_ - o.x; p.y() = row_ - o.y;

				if (p.dot(p0)>0)
				{
					m_middepth8u.at<uchar>(row_, col_) = 0;
				}
				else
				{
					indicator_2[NUM_indicator_2++] = indicator[i];
				}
			}

			circle(m_middepth8u, Point2i(depthSpaceElbowPosition.X, depthSpaceElbowPosition.Y), 3, Scalar(255, 255, 255), 2);

			circle(m_middepth8u, cv::Point2i(cx, cy), 3, Scalar(0, 0, 0), 2);
			circle(m_middepth8u, cv::Point2i(cx, cy), R, Scalar(0, 0, 0), 1);

			line(m_middepth8u, cv::Point2i(cx, cy), cv::Point2i(cx, cy) + 0.05f*Point(eigen_vecs[0].x*eigen_val[0], eigen_vecs[0].y*eigen_val[0]), Scalar(0, 0, 0));
			line(m_middepth8u, cv::Point2i(cx, cy), cv::Point2i(cx, cy) + 0.05f*Point(eigen_vecs[1].x*eigen_val[1], eigen_vecs[1].y*eigen_val[1]), Scalar(0, 0, 0));


			line(m_middepth8u, cv::Point2i(cx, cy) + Point(eigen_vecs[0].x*1.2f*R, eigen_vecs[0].y*1.2f*R)
				, cv::Point2i(cx, cy) + Point(eigen_vecs[0].x*1.2f*R, eigen_vecs[0].y*1.2f*R) + Point(eigen_vecs[1].x*eigen_val[1], eigen_vecs[1].y*eigen_val[1])
				, Scalar(0, 0, 0));

			pointcloud->DepthMatToPointCloud(depth_image[BACK_BUFFER], indicator_2, NUM_indicator_2);


			pointcloud_vector[BACK_BUFFER].swap(pointcloud->pointcloud_vector);
			pointcloud_center[BACK_BUFFER] << pointcloud->PointCloud_center_x, pointcloud->PointCloud_center_y, pointcloud->PointCloud_center_z;


			// Lock the mutex and swap the buffers
			{
				std::unique_lock<std::mutex> lock(swap_mutex);
				condition.wait(lock, [] {return main_released; });
				thread_released = false;

				depth_image[FRONT_BUFFER] = depth_image[BACK_BUFFER].clone();
				//color_image[FRONT_BUFFER] = color_image[BACK_BUFFER].clone();
				sensor_silhouette_buffer = m_middepth8u.clone();

				pointcloud_vector[FRONT_BUFFER].swap(pointcloud_vector[BACK_BUFFER]);
				pointcloud_center[FRONT_BUFFER] << pointcloud_center[BACK_BUFFER];

				thread_released = true;
				lock.unlock();
				condition.notify_one();
			}
			/*circle(m_middepth8u, cv::Point2i(cx, cy), 3, Scalar(0, 0, 0), 2);
			circle(m_middepth8u, cv::Point2i(cx, cy), R, Scalar(0, 0, 0), 1);

			line(m_middepth8u, cv::Point2i(cx, cy), cv::Point2i(cx, cy) + 0.05f*Point(eigen_vecs[0].x*eigen_val[0], eigen_vecs[0].y*eigen_val[0]), Scalar(0, 0, 0));
			line(m_middepth8u, cv::Point2i(cx, cy), cv::Point2i(cx, cy) + 0.05f*Point(eigen_vecs[1].x*eigen_val[1], eigen_vecs[1].y*eigen_val[1]), Scalar(0, 0, 0));


			line(m_middepth8u, cv::Point2i(cx, cy) + Point(eigen_vecs[0].x*1.2f*R, eigen_vecs[0].y*1.2f*R)
			, cv::Point2i(cx, cy) + Point(eigen_vecs[0].x*1.2f*R, eigen_vecs[0].y*1.2f*R) + Point(eigen_vecs[1].x*eigen_val[1], eigen_vecs[1].y*eigen_val[1])
			, Scalar(0, 0, 0));*/

		}
		else
		{
			continue;
		}
	}

	cout << "for no reason the run() breakdown!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
}