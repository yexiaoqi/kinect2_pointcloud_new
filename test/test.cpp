#include "stdafx.h"



#if 0

/*
将深度图映射到彩色图上，生成和深度图匹配的对齐彩色图,效果不是很好清晰度不不够
*/
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/LU>
#include <thread>

using namespace cv;
using namespace std;

struct KinectParm
{
	float fx_rgb;
	float fy_rgb;
	float cx_rgb;
	float cy_rgb;

	float fx_ir;
	float fy_ir;
	float cx_ir;
	float cy_ir;

	Eigen::Matrix3f R_ir2rgb;
	Eigen::Vector3f T_ir2rgb;
};

bool loadParm(KinectParm* kinectParm)
{
	// 加载参数
	ifstream parm("registration.txt");
	string stringLine;
	if (parm.is_open())
	{
		// rgb相机参数：fx,fy,cx,cy
		getline(parm, stringLine);
		stringstream lin(stringLine);
		string s1, s2, s3, s4, s5, s6, s7, s8, s9;
		lin >> s1 >> s2 >> s3 >> s4;
		kinectParm->fx_rgb = atof(s1.c_str());
		kinectParm->fy_rgb = atof(s2.c_str());
		kinectParm->cx_rgb = atof(s3.c_str());
		kinectParm->cy_rgb = atof(s4.c_str());
		stringLine.clear();
		// ir相机参数：fx,fy,cx,cy
		getline(parm, stringLine);
		stringstream lin2(stringLine);
		lin2 << stringLine;
		lin2 >> s1 >> s2 >> s3 >> s4;
		kinectParm->fx_ir = atof(s1.c_str());
		kinectParm->fy_ir = atof(s2.c_str());
		kinectParm->cx_ir = atof(s3.c_str());
		kinectParm->cy_ir = atof(s4.c_str());
		stringLine.clear();

		// R_ir2rgb
		getline(parm, stringLine);
		stringstream lin3(stringLine);
		lin3 << stringLine;
		lin3 >> s1 >> s2 >> s3 >> s4 >> s5 >> s6 >> s7 >> s8 >> s9;
		kinectParm->R_ir2rgb <<
			atof(s1.c_str()), atof(s2.c_str()), atof(s3.c_str()),
			atof(s4.c_str()), atof(s5.c_str()), atof(s6.c_str()),
			atof(s7.c_str()), atof(s8.c_str()), atof(s9.c_str());
		stringLine.clear();

		// T_ir2rgb
		getline(parm, stringLine);
		stringstream lin4(stringLine);
		lin4 << stringLine;
		lin4 >> s1 >> s2 >> s3;
		kinectParm->T_ir2rgb << atof(s1.c_str()), atof(s2.c_str()), atof(s3.c_str());
	}
	else
	{
		cout << "parm.txt not right!!!";
		return false;
	}
	cout << "******************************************" << endl;

	cout << "fx_rgb:    " << kinectParm->fx_rgb << endl;
	cout << "fy_rgb:    " << kinectParm->fy_rgb << endl;
	cout << "cx_rgb:    " << kinectParm->cx_rgb << endl;
	cout << "cy_rgb:    " << kinectParm->cy_rgb << endl;
	cout << "******************************************" << endl;
	cout << "fx_ir:     " << kinectParm->fx_ir << endl;
	cout << "fy_ir:     " << kinectParm->fy_ir << endl;
	cout << "cx_ir:     " << kinectParm->cx_ir << endl;
	cout << "cy_ir:     " << kinectParm->cy_ir << endl;
	cout << "******************************************" << endl;
	cout << "R_ir2rgb:" << endl << kinectParm->R_ir2rgb << endl;
	cout << "******************************************" << endl;
	cout << "T_ir2rgb:" << endl << kinectParm->T_ir2rgb << endl;
	cout << "******************************************" << endl;
	return true;
}




int main()
{
	// 1. 读取参数
	KinectParm *parm = new KinectParm();
	if (!loadParm(parm))
		return 0;
	// 2. 载入rgb图片和深度图并显示
	Mat bgr(1080, 1920, CV_8UC3);
	bgr = imread("C:/vsprojects/test/test/result2/rgb/1.png");
	Mat depth(424, 512, CV_16UC1);
	depth = imread("C:/vsprojects/test/test/result2/depth/1.png", IMREAD_ANYDEPTH);   // 图片读入后的格式不一定和定义时候的一样，比如这里读入后的格式就是8UC3
	Mat depth2rgb = imread("depth2rgb.jpg");
	// 3. 显示
	thread th = std::thread([&] {
		while (true)
		{
			imshow("原始彩色图", bgr);
			waitKey(1);
			imshow("原始深度图", depth);
			waitKey(1);
			imshow("原始投影图", depth2rgb);
			waitKey(1);
		}
	});
	// 4. 变换

	// 4.1 计算各个矩阵
#pragma region  非齐次
	Eigen::Matrix3f K_ir;           // ir内参矩阵
	K_ir <<
		parm->fx_ir, 0, parm->cx_ir,
		0, parm->fy_ir, parm->cy_ir,
		0, 0, 1;
	Eigen::Matrix3f K_rgb;          // rgb内参矩阵
	K_rgb <<
		parm->fx_rgb, 0, parm->cx_rgb,
		0, parm->fy_rgb, parm->cy_rgb,
		0, 0, 1;

	Eigen::Matrix3f R;
	Eigen::Vector3f T;
	R = K_rgb*parm->R_ir2rgb*K_ir.inverse();
	T = K_rgb*parm->T_ir2rgb;

	cout << "K_rgb:\n" << K_rgb << endl;
	cout << "K_ir:\n" << K_ir << endl;
	cout << "R:\n" << R << endl;
	cout << "T:\n" << T << endl;

	cout << depth.type() << endl;


	// 4.2 计算投影
	Mat result(424, 512, CV_8UC3);
	int i = 0;
	for (int row = 0; row < 424; row++)
	{
		for (int col = 0; col < 512; col++)
		{
			unsigned short* p = (unsigned short*)depth.data;
			unsigned short depthValue = p[row * 512 + col];
			//cout << "depthValue       " << depthValue << endl;
			if (depthValue != -std::numeric_limits<unsigned short>::infinity() && depthValue != -std::numeric_limits<unsigned short>::infinity() && depthValue != 0 && depthValue != 65535)
			{
				// 投影到彩色图上的坐标
				Eigen::Vector3f uv_depth(col, row, 1.0f);                            // !!!p_ir
				Eigen::Vector3f uv_color = depthValue / 1000.f*R*uv_depth + T / 1000;   // !!!Z_rgb*p_rgb=R*Z_ir*p_ir+T; (除以1000，是为了从毫米变米)

				int X = static_cast<int>(uv_color[0] / uv_color[2]);                // !!!Z_rgb*p_rgb -> p_rgb
				int Y = static_cast<int>(uv_color[1] / uv_color[2]);                // !!!Z_rgb*p_rgb -> p_rgb
																					//cout << "X:       " << X << "     Y:      " << Y << endl;
				if ((X >= 0 && X < 1920) && (Y >= 0 && Y < 1080))
				{
					//cout << "X:       " << X << "     Y:      " << Y << endl;
					result.data[i * 3] = bgr.data[3 * (Y * 1920 + X)];
					result.data[i * 3 + 1] = bgr.data[3 * (Y * 1920 + X) + 1];
					result.data[i * 3 + 2] = bgr.data[3 * (Y * 1920 + X) + 2];
				}
				else
				{
					result.data[i * 3] = 0;
					result.data[i * 3 + 1] = 0;
					result.data[i * 3 + 2] = 0;
				}
			}
			else
			{
				result.data[i * 3] = 0;
				result.data[i * 3 + 1] = 0;
				result.data[i * 3 + 2] = 0;
			}
			i++;
		}
	}
	imwrite("registrationResult2.png", result);
	thread th2 = std::thread([&] {
		while (true)
		{
			imshow("结果图", result);
			waitKey(1);
		}
	});

	th.join();
	th2.join();
#pragma endregion


	system("pause");
	return 0;
}

#endif

























#if 0
//深度图和rgb图转化为点云,现在改成了直接存为ply而不是pcd格式
#include<iostream>

#include<string>

using namespace std;

//opencv库
#include <opencv2/opencv.hpp>
//#include<opencv2/core/core.cpp>

//#include<opencv2/highgui/highgui.hpp>

//PCL库

#include<pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include<pcl/point_types.h>
#include <pcl/point_cloud.h>





//定义点云类型

typedef pcl::PointXYZRGBA PointT;

typedef pcl::PointCloud<PointT> PointCloud;

//相机内参
////
const double camera_factor = 1000;

const double camera_cx = 254.616;

const double camera_cy = 207.801;

const double camera_fx = 364.547;

const double camera_fy = 364.547;
//const double camera_factor = 1000;
//
//const double camera_cx = 325.5;
//
//const double camera_cy = 253.5;
//
//const double camera_fx = 518.0;
//
//const double camera_fy = 519.0;
//const double camera_factor = 1;
//
//const double camera_cx = 200;
//
//const double camera_cy = 200;
//const double camera_fx = 1000;
//
//const double camera_fy = 1000;

//const double camera_fx = 367.749;
//
//const double camera_fy = 367.749;
//const double camera_fx = 284;
//
//const double camera_fy = 284;

//主函数

int main(int argc, char** argv)

{

	//读取./data/rgb.png和./data/depth.png，并转化为点云

	//图像矩阵

	cv::Mat rgb, depth;

	//使用cv::imread()来读取图像

	//rgb图像是8UC3的彩色图像

	rgb = cv::imread("C:/vsprojects/test/test/inputimg190519/rgb/1.png");

	//depth是16UC1的单通道图像，注意flags设置为-1，表示读取原始数据不做修改

	/*depth = cv::imread("C:/vsprojects/cvtest/cvtest/190509yqy/dt_1.png", -1);*/
	depth = cv::imread("C:/vsprojects/test/test/inputimg190519/depth/1.png", -1);
	//rgb = cv::imread("C:/vsprojects/cvtest/cvtest/20170907/group1/color_map/frame_000001.png");

	////depth是16UC1的单通道图像，注意flags设置为-1，表示读取原始数据不做修改

	//depth = cv::imread("C:/vsprojects/cvtest/cvtest/20170907/group1/depth_map/frame_000001.png", -1);

	//点云变量

	//使用智能指针，创建一个空点云。这种指针用完会自动释放

	//PointCloud::Ptr cloud(new PointCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//遍历深度图

	for (int m = 0; m<depth.rows; m++)

		for (int n = 0; n<depth.cols; n++)

		{

			//获取深度图中(m,n)处的值

			ushort d = depth.ptr<ushort>(m)[n];

			//d可能没有值，若如此，跳过此点

			if (d == 0)

				continue;

			//d存在值，则向点云增加一个点

			//PointT p;
			pcl::PointXYZRGB p;

			//计算这个点的空间坐标

			p.z = double(d) / camera_factor;

			p.x = (n - camera_cx)*p.z / camera_fx;

			p.y = (m - camera_cy)*p.z / camera_fy;

			//从rgb图像中获取它的颜色

			//rgb是三通道的BGR格式图，所以按下面的顺序获取颜色

			p.b = rgb.ptr<uchar>(m)[n * 3];

			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];

			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

			//把p加入到点云中

			cloud->points.push_back(p);

		}

	//设置并保存点云

	cloud->height = 1;

	cloud->width = cloud->points.size();

	cout << "point cloud size=" << cloud->points.size() << endl;

	cloud->is_dense = false;

	//pcl::io::savePCDFile("C:/vsprojects/cvtest/cvtest/pointcloudyqy190509input.pcd", *cloud);
	pcl::PLYWriter writer;
	writer.write("C:/vsprojects/test/test/kinectinput190519.ply", *cloud);

	//清楚数据并保存

	cloud->points.clear();

	cout << "Point cloud saved." << endl;

	return 0;

}
#endif



#if 0
//pcd转ply文件成功
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include<pcl/PCLPointCloud2.h>
#include<iostream>
#include<string>

using namespace pcl;
using namespace pcl::io;
using namespace std;

int PCDtoPLYconvertor(string & input_filename, string& output_filename)
{
	pcl::PCLPointCloud2 cloud;
	if (loadPCDFile(input_filename, cloud) < 0)
	{
		cout << "Error: cannot load the PCD file!!!" << endl;
		return -1;
	}
	PLYWriter writer;
	writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true, true);
	return 0;

}

int main()
{
	//string input_filename = "C:/vsprojects/cvtest/cvtest/pointcloud.pcd";
	//string output_filename = "C:/vsprojects/cvtest/cvtest/pointcloud.ply";
	string input_filename = "C:/vsprojects/cvtest/cvtest/pointcloudyqy1905092.pcd";
	string output_filename = "C:/vsprojects/cvtest/cvtest/pointcloudyqy1905092.ply";
	PCDtoPLYconvertor(input_filename, output_filename);
	return 0;
}
#endif



#if 0
/*
将深度图映射到彩色图上，生成和深度图匹配的对齐彩色图,效果不是很好清晰度不不够
*/
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/LU>
#include <thread>

using namespace cv;
using namespace std;

struct KinectParm
{
	float fx_rgb;
	float fy_rgb;
	float cx_rgb;
	float cy_rgb;

	float fx_ir;
	float fy_ir;
	float cx_ir;
	float cy_ir;

	Eigen::Matrix3f R_ir2rgb;
	Eigen::Vector3f T_ir2rgb;
};

bool loadParm(KinectParm* kinectParm)
{
	// 加载参数
	ifstream parm("registration.txt");
	string stringLine;
	if (parm.is_open())
	{
		// rgb相机参数：fx,fy,cx,cy
		getline(parm, stringLine);
		stringstream lin(stringLine);
		string s1, s2, s3, s4, s5, s6, s7, s8, s9;
		lin >> s1 >> s2 >> s3 >> s4;
		kinectParm->fx_rgb = atof(s1.c_str());
		kinectParm->fy_rgb = atof(s2.c_str());
		kinectParm->cx_rgb = atof(s3.c_str());
		kinectParm->cy_rgb = atof(s4.c_str());
		stringLine.clear();
		// ir相机参数：fx,fy,cx,cy
		getline(parm, stringLine);
		stringstream lin2(stringLine);
		lin2 << stringLine;
		lin2 >> s1 >> s2 >> s3 >> s4;
		kinectParm->fx_ir = atof(s1.c_str());
		kinectParm->fy_ir = atof(s2.c_str());
		kinectParm->cx_ir = atof(s3.c_str());
		kinectParm->cy_ir = atof(s4.c_str());
		stringLine.clear();

		// R_ir2rgb
		getline(parm, stringLine);
		stringstream lin3(stringLine);
		lin3 << stringLine;
		lin3 >> s1 >> s2 >> s3 >> s4 >> s5 >> s6 >> s7 >> s8 >> s9;
		kinectParm->R_ir2rgb <<
			atof(s1.c_str()), atof(s2.c_str()), atof(s3.c_str()),
			atof(s4.c_str()), atof(s5.c_str()), atof(s6.c_str()),
			atof(s7.c_str()), atof(s8.c_str()), atof(s9.c_str());
		stringLine.clear();

		// T_ir2rgb
		getline(parm, stringLine);
		stringstream lin4(stringLine);
		lin4 << stringLine;
		lin4 >> s1 >> s2 >> s3;
		kinectParm->T_ir2rgb << atof(s1.c_str()), atof(s2.c_str()), atof(s3.c_str());
	}
	else
	{
		cout << "parm.txt not right!!!";
		return false;
	}
	cout << "******************************************" << endl;

	cout << "fx_rgb:    " << kinectParm->fx_rgb << endl;
	cout << "fy_rgb:    " << kinectParm->fy_rgb << endl;
	cout << "cx_rgb:    " << kinectParm->cx_rgb << endl;
	cout << "cy_rgb:    " << kinectParm->cy_rgb << endl;
	cout << "******************************************" << endl;
	cout << "fx_ir:     " << kinectParm->fx_ir << endl;
	cout << "fy_ir:     " << kinectParm->fy_ir << endl;
	cout << "cx_ir:     " << kinectParm->cx_ir << endl;
	cout << "cy_ir:     " << kinectParm->cy_ir << endl;
	cout << "******************************************" << endl;
	cout << "R_ir2rgb:" << endl << kinectParm->R_ir2rgb << endl;
	cout << "******************************************" << endl;
	cout << "T_ir2rgb:" << endl << kinectParm->T_ir2rgb << endl;
	cout << "******************************************" << endl;
	return true;
}


int countdepth = 1;
int countrgb = 1;

int main()
{


	// 1. 读取参数
	KinectParm *parm = new KinectParm();
	if (!loadParm(parm))
		return 0;
	// 4.1 计算各个矩阵
	//#pragma region  非齐次
	Eigen::Matrix3f K_ir;           // ir内参矩阵
	K_ir <<
		parm->fx_ir, 0, parm->cx_ir,
		0, parm->fy_ir, parm->cy_ir,
		0, 0, 1;
	Eigen::Matrix3f K_rgb;          // rgb内参矩阵
	K_rgb <<
		parm->fx_rgb, 0, parm->cx_rgb,
		0, parm->fy_rgb, parm->cy_rgb,
		0, 0, 1;

	Eigen::Matrix3f R;
	Eigen::Vector3f T;
	R = K_rgb*parm->R_ir2rgb*K_ir.inverse();
	T = K_rgb*parm->T_ir2rgb;

	cout << "K_rgb:\n" << K_rgb << endl;
	cout << "K_ir:\n" << K_ir << endl;
	cout << "R:\n" << R << endl;
	cout << "T:\n" << T << endl;

	

	while (true)
	{
		
		// 2. 载入rgb图片和深度图并显示
		Mat bgr(1080, 1920, CV_8UC4);
		std::stringstream str1;
		if (countrgb < 21)
		{
			str1 << "C:/vsprojects/cvtest/cvtest/result/rgb/" << countrgb << ".png";
			countrgb++;
		}
		//bgr = imread(str1.str());
		bgr = imread("C:/vsprojects/cvtest/cvtest/result/rgb/1.png");
		Mat depth(424, 512, CV_16UC1);
		std::stringstream str2;
		if (countdepth < 21)
		{
			str2 << "C:/vsprojects/cvtest/cvtest/result/depth/" << countdepth << ".png";
			countdepth++;
		}
		//depth = imread(str2.str(), IMREAD_ANYDEPTH);
		depth = imread("C:/vsprojects/cvtest/cvtest/result/depth/1.png", IMREAD_ANYDEPTH);   // 图片读入后的格式不一定和定义时候的一样，比如这里读入后的格式就是8UC3
		Mat depth2rgb = imread("depth2rgb.jpg");
		// 3. 显示
		thread th = std::thread([&] {
			while (true)
			{
				imshow("原始彩色图", bgr);
				waitKey(1);
				imshow("原始深度图", depth);
				waitKey(1);
				imshow("原始投影图", depth2rgb);
				waitKey(1);
			}
		});
		// 4. 变换
		cout << depth.type() << endl;
		

		// 4.2 计算投影
		Mat result(424, 512, CV_8UC3);
		int i = 0;
		for (int row = 0; row < 424; row++)
		{
			for (int col = 0; col < 512; col++)
			{
				unsigned short* p = (unsigned short*)depth.data;
				unsigned short depthValue = p[row * 512 + col];
				//cout << "depthValue       " << depthValue << endl;
				if (depthValue != -std::numeric_limits<unsigned short>::infinity() && depthValue != -std::numeric_limits<unsigned short>::infinity() && depthValue != 0 && depthValue != 65535)
				{
					// 投影到彩色图上的坐标
					Eigen::Vector3f uv_depth(col, row, 1.0f);                            // !!!p_ir
					Eigen::Vector3f uv_color = depthValue / 1000.f*R*uv_depth + T / 1000;   // !!!Z_rgb*p_rgb=R*Z_ir*p_ir+T; (除以1000，是为了从毫米变米)

					int X = static_cast<int>(uv_color[0] / uv_color[2]);                // !!!Z_rgb*p_rgb -> p_rgb
					int Y = static_cast<int>(uv_color[1] / uv_color[2]);                // !!!Z_rgb*p_rgb -> p_rgb
																						//cout << "X:       " << X << "     Y:      " << Y << endl;
					if ((X >= 0 && X < 1920) && (Y >= 0 && Y < 1080))
					{
						//cout << "X:       " << X << "     Y:      " << Y << endl;
						result.data[i * 3] = bgr.data[3 * (Y * 1920 + X)];
						result.data[i * 3 + 1] = bgr.data[3 * (Y * 1920 + X) + 1];
						result.data[i * 3 + 2] = bgr.data[3 * (Y * 1920 + X) + 2];
					}
					else
					{
						result.data[i * 3] = 0;
						result.data[i * 3 + 1] = 0;
						result.data[i * 3 + 2] = 0;
					}
				}
				else
				{
					result.data[i * 3] = 0;
					result.data[i * 3 + 1] = 0;
					result.data[i * 3 + 2] = 0;
				}
				i++;
			}
		}


		std::stringstream str3;
		str3 << "C:/vsprojects/cvtest/cvtest/result/rgb2/" << countrgb << ".png";
		imwrite(str3.str(), result);

		//imwrite("registrationResult.png", result);
		thread th2 = std::thread([&] {
			while (true)
			{
				imshow("结果图", result);
				waitKey(1);
			}
		});

		th.join();
		th2.join();
	}
//#pragma endregion


	system("pause");
	return 0;
}
#endif




#if 1
//生成点云成功,但保存的颜色有问题
#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <kinect.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGB PointType;

int main(int argc, char* argv[])
{
	// PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
		new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);

	// Point Cloud
	pcl::PointCloud<PointType>::ConstPtr cloud;

	// Retrieved Point Cloud Callback Function
	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
		[&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
		boost::mutex::scoped_lock lock(mutex);

		/* Point Cloud Processing */

		cloud = ptr->makeShared();
	};
	// Kinect2Grabber
	//  boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();
	boost::shared_ptr<pcl::Grabber> grabber = boost::shared_ptr<pcl::Grabber>(new pcl::Kinect2Grabber);
	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);

	// Start Grabber
	grabber->start();
	int count = 0;
	while (!viewer->wasStopped()) {
		// Update Viewer
		viewer->spinOnce();

		boost::mutex::scoped_try_lock lock(mutex);

		/*pcl::PLYWriter writer;
		writer.write("kinectcloud190519.ply", *cloud);*/
		if (count == 0)
		{
			pcl::PLYWriter writer;
			writer.write("kinectcloud190519.ply", *cloud);
			
			//pcl::io::savePCDFileASCII("kinectcloud190519.ply",*cloud);
			count += 1;
		}
		
		if (lock.owns_lock() && cloud) {
			// Update Point Cloud
			if (!viewer->updatePointCloud(cloud, "cloud")) {
				viewer->addPointCloud(cloud, "cloud");
				//if (count == 0)
				//{
				//	
				//	pcl::io::savePCDFileASCII("kinectcloud1905192.ply", *cloud);
				//	//count += 1;
				//}

			}
		}
	}
	// Stop Grabber
	grabber->stop();

	// Disconnect Callback Function
	if (connection.connected()) {
		connection.disconnect();
	}

	return 0;
}
#endif









#if 0
//生成点云失败
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <kinect.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace cv;
IKinectSensor* pSensor = nullptr;
ICoordinateMapper* pMapper = nullptr;
const int iWidth = 512, iHeight = 424;
CameraSpacePoint depth2xyz[iWidth*iHeight];
ColorSpacePoint depth2rgb[iWidth*iHeight];
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{

	viewer.setBackgroundColor(1, 1, 1);//设置背景颜色 


}

bool initKinect()
{
	if (FAILED(GetDefaultKinectSensor(&pSensor))) return false;
	if (pSensor)
	{
		pSensor->get_CoordinateMapper(&pMapper);
		pSensor->Open();
		return true;
	}
	else return false;
}

void getPointCloudFromImage(Mat depthImage, Mat rgbImage, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
{







	pMapper->MapDepthFrameToCameraSpace(iWidth*iHeight, reinterpret_cast<UINT16*>(depthImage.data), iWidth*iHeight, depth2xyz);
	pMapper->MapDepthFrameToColorSpace(512 * 424, reinterpret_cast<UINT16*>(depthImage.data), 512 * 424, depth2rgb);

	cloud_out.height = 1;
	cloud_out.is_dense = 1;

	for (size_t i = 0; i < iWidth; i++)
	{
		for (size_t j = 0; j < iHeight; j++)
		{
			pcl::PointXYZRGB pointTemp;
			if (depth2xyz[i + j*iWidth].Z > 0.5)
			{

				pointTemp.x = depth2xyz[i + j*iWidth].X;
				pointTemp.y = depth2xyz[i + j*iWidth].Y;
				pointTemp.z = depth2xyz[i + j*iWidth].Z;
				int X = static_cast<int>(depth2rgb[j * 512 + i].X);
				int Y = static_cast<int>(depth2rgb[j * 512 + i].Y);
				if (X > 0 && Y > 0 && X < 1920 && Y < 1080)
				{
					Vec3b* pixelsRGBImage = rgbImage.ptr<Vec3b>(Y);
					pointTemp.g = pixelsRGBImage[X][0];
					pointTemp.b = pixelsRGBImage[X][1];
					pointTemp.r = pixelsRGBImage[X][2];
					cloud_out.push_back(pointTemp);
				}
				else continue;

			}
		}
	}
}

int main()
{
	initKinect();
	Mat rgbImage = imread("C:/vsprojects/cvtest/cvtest/result/rgb/1.png");
	Mat depthImage = imread("C:/vsprojects/cvtest/cvtest/result/depth/1.png", -1);






	IDepthFrame* m_pDepthFrame;
	IColorFrame* m_pColorFrame;
	// 三个图片格式
	Mat i_rgb(1080, 1920, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
	Mat i_depth(424, 512, CV_8UC1);
	//Mat i_rgb(480,640, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
	//Mat i_depth(480, 640, CV_8UC1);
	Mat i_ir(424, 512, CV_16UC1);

	UINT16 *depthData = new UINT16[424 * 512];
	IMultiSourceFrame* m_pMultiFrame = nullptr;
	int sample_id = 1;
	while (true)
	{
		// 获取新的一个多源数据帧

		hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
		if (FAILED(hr) || !m_pMultiFrame)
		{
			cout << "!!!" << endl;
			continue;
		}


		// 从多源数据帧中分离出彩色数据，深度数据和红外数据
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_ColorFrameReference(&m_pColorFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pColorFrameReference->AcquireFrame(&m_pColorFrame);
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_DepthFrameReference(&m_pDepthFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pDepthFrameReference->AcquireFrame(&m_pDepthFrame);



		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
		getPointCloudFromImage(depthImage, rgbImage, *cloud_out);
		pcl::visualization::CloudViewer viewerG("Cloud Viewe");
		//pcl::io::savePCDFileASCII("输出点云.pcd", *cloud_out);
		pcl::PLYWriter writer;
		writer.write("out190509.ply", *cloud_out);
		viewerG.runOnVisualizationThreadOnce(viewerOneOff);
		viewerG.showCloud(cloud_out);
		while (true) if (cv::waitKey(30) == VK_ESCAPE) break;
		return 0;
	}
#endif
 





#if 0
//一个尝试解决深度图墙面截断，点云存储有点问题
#include <kinect.h>
#include <iostream>
#include <opencv2\opencv.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <kinect.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include<string>

	using namespace cv;
	using namespace std;

	typedef pcl::PointXYZRGBA PointT;

	typedef pcl::PointCloud<PointT> PointCloud;
	template<class Interface>

	inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}
	// 转换depth图像到cv::Mat
	cv::Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
	{



		cv::Mat img(nHeight, nWidth, CV_16UC1);
		UINT16* p_mat = (UINT16*)img.data;

		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		while (pBuffer < pBufferEnd)
		{
			*p_mat = *pBuffer;
			p_mat++;
			++pBuffer;
		}

	




		////cv::Mat img(nHeight, nWidth, CV_8UC3);
		//cv::Mat img(nHeight, nWidth, CV_16UC1);
		//uchar* p_mat = img.data;

		//const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		//while (pBuffer < pBufferEnd)
		//{
		//	USHORT depth = *pBuffer;

		//	BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

		//	*p_mat = intensity;
		//	p_mat++;
		//	*p_mat = intensity;
		//	p_mat++;
		//	*p_mat = intensity;
		//	p_mat++;

		//	++pBuffer;
		//}
		return img;
	}
	// 转换color图像到cv::Mat
	cv::Mat ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight)
	{








		cv::Mat img(nHeight, nWidth, CV_8UC3);
		uchar* p_mat = img.data;

		const RGBQUAD* pBufferEnd = pBuffer + (nWidth * nHeight);

		while (pBuffer < pBufferEnd)
		{
			*p_mat = pBuffer->rgbBlue;
			p_mat++;
			*p_mat = pBuffer->rgbGreen;
			p_mat++;
			*p_mat = pBuffer->rgbRed;
			p_mat++;

			++pBuffer;
		}
		return img;
	}


	void main()
	{
		////////////////////////////////////////////////////////////////
		int depth_width = 512; //depth图像就是这么小
		int depth_height = 424;
		int color_widht = 1920; //color图像就是辣么大
		int color_height = 1080;

		//cv::Mat depthImg_show = cv::Mat::zeros(depth_height, depth_width, CV_8UC3);//原始UINT16 深度图像不适合用来显示，所以需要砍成8位的就可以了，但是显示出来也不是非常好，最好能用原始16位图像颜色编码，凑合着看了
		cv::Mat depthImg_show = cv::Mat::zeros(depth_height, depth_width, CV_16UC1);//the depth image
		cv::Mat colorImg = cv::Mat::zeros(color_height, color_widht, CV_8UC3);//the color image
																			  // Current Kinect
		IKinectSensor* m_pKinectSensor = NULL;
		// Depth reader
		IDepthFrameReader*  m_pDepthFrameReader = NULL;
		// Color reader
		IColorFrameReader*  m_pColorFrameReader = NULL;
		RGBQUAD* m_pColorRGBX = new RGBQUAD[color_widht * color_height];
		//open it!
		HRESULT hr;

		hr = GetDefaultKinectSensor(&m_pKinectSensor);
		if (FAILED(hr))
		{
			cout << "FUCK! Can not find the Kinect!" << endl;
			cv::waitKey(0);
			exit(0);
		}

		if (m_pKinectSensor)
		{
			// Initialize the Kinect and get the depth reader
			IDepthFrameSource* pDepthFrameSource = NULL;

			hr = m_pKinectSensor->Open();

			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
			}

			SafeRelease(pDepthFrameSource);

			// for color
			// Initialize the Kinect and get the color reader
			IColorFrameSource* pColorFrameSource = NULL;
			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
			}

			if (SUCCEEDED(hr))
			{
				hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
			}

			SafeRelease(pColorFrameSource);
		}

		//valify the depth reader
		if (!m_pDepthFrameReader)
		{
			cout << "FUCK! Can not find the m_pDepthFrameReader!" << endl;
			cv::waitKey(0);
			exit(0);
		}
		//valify the color reader
		if (!m_pDepthFrameReader)
		{
			cout << "FUCK! Can not find the m_pColorFrameReader!" << endl;
			cv::waitKey(0);
			exit(0);
		}
		// get the data!
		UINT nBufferSize_depth = 0;
		UINT16 *pBuffer_depth = NULL;
		UINT nBufferSize_coloar = 0;
		RGBQUAD *pBuffer_color = NULL;

		char key = 0;

		while (true) // 貌似要一直尝试，不一定每次都能读取到图像
		{
			IDepthFrame* pDepthFrame = NULL;
			HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
			if (SUCCEEDED(hr))
			{
				USHORT nDepthMinReliableDistance = 0;
				USHORT nDepthMaxReliableDistance = 0;
				if (SUCCEEDED(hr))
				{
					hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
				}

				if (SUCCEEDED(hr))
				{
					hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
				}
				if (SUCCEEDED(hr))
				{
					hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);
					depthImg_show = ConvertMat(pBuffer_depth, depth_width, depth_height, nDepthMinReliableDistance, nDepthMaxReliableDistance);
				}
			}
			SafeRelease(pDepthFrame);


			//for color
			IColorFrame* pColorFrame = NULL;
			hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
			ColorImageFormat imageFormat = ColorImageFormat_None;
			if (SUCCEEDED(hr))
			{
				ColorImageFormat imageFormat = ColorImageFormat_None;
				if (SUCCEEDED(hr))
				{
					hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
				}
				if (SUCCEEDED(hr))
				{
					hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
				}
				if (SUCCEEDED(hr))
				{
					if (imageFormat == ColorImageFormat_Bgra)//这里有两个format，不知道具体含义，大概一个预先分配内存，一个需要自己开空间吧
					{
						hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize_coloar, reinterpret_cast<BYTE**>(&pBuffer_color));
					}
					else if (m_pColorRGBX)
					{
						pBuffer_color = m_pColorRGBX;
						nBufferSize_coloar = color_widht * color_height * sizeof(RGBQUAD);
						hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize_coloar, reinterpret_cast<BYTE*>(pBuffer_color), ColorImageFormat_Bgra);
	}
					else
					{
						hr = E_FAIL;
					}
					colorImg = ConvertMat(pBuffer_color, color_widht, color_height);
			}
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				//遍历深度图

				for (int m = 0; m<depthImg_show.rows; m++)

					for (int n = 0; n<depthImg_show.cols; n++)

					{

						//获取深度图中(m,n)处的值

						ushort d = depthImg_show.ptr<ushort>(m)[n];

						//d可能没有值，若如此，跳过此点

						if (d == 0)

							continue;

						//d存在值，则向点云增加一个点

						//PointT p;
						pcl::PointXYZRGB p;

						//计算这个点的空间坐标

						//p.z = double(d); 
						p.z = double(d) / 500;
						p.x = n;

						p.y = m;

						/*p.x = (n - camera_cx)*p.z / camera_fx;

						p.y = (m - camera_cy)*p.z / camera_fy;*/

						//从rgb图像中获取它的颜色

						//rgb是三通道的BGR格式图，所以按下面的顺序获取颜色

						/*p.b = i_rgb.ptr<uchar>(m)[n * 3];

						p.g = i_rgb.ptr<uchar>(m)[n * 3 + 1];

						p.r = i_rgb.ptr<uchar>(m)[n * 3 + 2];*/

						//把p加入到点云中

						cloud->points.push_back(p);

					}

				//设置并保存点云

				cloud->height = 1;

				cloud->width = cloud->points.size();

				cout << "point cloud size=" << cloud->points.size() << endl;

				cloud->is_dense = false;

				//pcl::io::savePCDFile("C:/vsprojects/cvtest/cvtest/pointcloudyqy190509input.pcd", *cloud);
				pcl::PLYWriter writer;
				writer.write("C:/vsprojects/cvtest/cvtest/pointcloudyqy190511.ply", *cloud);

				//清楚数据并保存

				cloud->points.clear();

				cout << "Point cloud saved." << endl;

				SafeRelease(pColorFrame);
		}

			cv::imshow("depth", depthImg_show);
			cv::imwrite("depth190512.png", depthImg_show);
			cv::imshow("color", colorImg);
			key = cv::waitKey(1);
			if (key == 27)
			{
				break;
			}
	}


		if (m_pColorRGBX)
		{
			delete[] m_pColorRGBX;
			m_pColorRGBX = NULL;
		}
		// close the Kinect Sensor
		if (m_pKinectSensor)
		{
			m_pKinectSensor->Close();
		}
		SafeRelease(m_pKinectSensor);
}
#endif



#if 0
	//尝试解决深度图中墙面截断问题，存为16位图像成功，增加内参后点云也对了
#include <stdio.h>

#include <kinect.h>
#include <iostream>
#include <opencv2\opencv.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <kinect.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include<string>
//定义点云类型

	typedef pcl::PointXYZRGBA PointT;

	typedef pcl::PointCloud<PointT> PointCloud;

	////相机内参
	const double camera_factor = 500;

	const double camera_cx = 256;

	const double camera_cy = 212;

	const double camera_fx = 367.749;

	const double camera_fy = 367.749;
	using namespace cv;

	// 转换depth图像到cv::Mat
	Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight)
	{


		cv::Mat img(nHeight, nWidth, CV_16UC1);
		UINT16* p_mat = (UINT16*)img.data;//yqy 重点！！！！！不是uchar而是UINT16

		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		while (pBuffer < pBufferEnd)
		{
			*p_mat = *pBuffer;
			p_mat++;
			++pBuffer;
		}



		//Mat img(nHeight, nWidth, CV_8UC1);
		//uchar* p_mat = img.data;//指向头指针

		//const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);//指向最后一个元素的指针

		//while (pBuffer < pBufferEnd)//16位最大值为65536
		//{
		//	//*p_mat++ = *pBuffer++ / 65536.0 * 256;
		//	*p_mat++ = *pBuffer++ ;
		//}
		return img;
	}
	int main()
	{
		IKinectSensor*          m_pKinectSensor;
		IDepthFrameReader*      m_pDepthFrameReader;
		IDepthFrame* pDepthFrame = NULL;
		IFrameDescription* pFrameDescription = NULL;
		IDepthFrameSource* pDepthFrameSource = NULL;

		HRESULT hr = GetDefaultKinectSensor(&m_pKinectSensor);//获取默认kinect传感器
		assert(hr >= 0);
		printf("打开kinect传感器成功\n");

		hr = m_pKinectSensor->Open();//打开传感器
		assert(hr >= 0);
		hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);//获得深度信息传感器
		assert(hr >= 0);
		hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);//打开深度信息帧读取器
		assert(hr >= 0);

		while (hr < 0 || pDepthFrame == NULL)
			hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//由于有时候获取不到，因此循环获取最近的帧

		assert(hr >= 0);
		hr = pDepthFrame->get_FrameDescription(&pFrameDescription);//获取帧的像素信息（宽和高）
		int depth_width, depth_height;
		pFrameDescription->get_Width(&depth_width);
		pFrameDescription->get_Height(&depth_height);
		printf("width=%d height=%d\n", depth_width, depth_height);

		USHORT nDepthMinReliableDistance = 0;//获取最大、最小深度距离信息
		USHORT nDepthMaxReliableDistance = 0;
		assert(hr >= 0);
		hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
		assert(hr >= 0);
		hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);

		printf("nDepthMinReliableDistance=%d nDepthMaxReliableDistance=%d\n", nDepthMinReliableDistance, nDepthMaxReliableDistance);

		UINT nBufferSize_depth = 0;
		UINT16 *pBuffer_depth = NULL;
		pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);//获取图像像素个数和指向图像的指针


																				//转换为MAT格式
		Mat i_depth = ConvertMat(pBuffer_depth, depth_width, depth_height);//转换为8位的mat


		//cv::equalizeHist(i_depth, i_depth);//均衡化，为了提高显示效果

		cv::imwrite("MyFirstKinectImg.png", i_depth);//保存图片
													   //用opencv显示

		cv::namedWindow("display");

		cv::imshow("display", i_depth);




		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		//遍历深度图

		for (int m = 0; m<i_depth.rows; m++)

			for (int n = 0; n<i_depth.cols; n++)

			{

				//获取深度图中(m,n)处的值

				ushort d = i_depth.ptr<ushort>(m)[n];

				//d可能没有值，若如此，跳过此点

				if (d == 0)

					continue;

				//d存在值，则向点云增加一个点

				//PointT p;
				pcl::PointXYZRGB p;

				//计算这个点的空间坐标

				//p.z = double(d); 
				p.z = double(d) / camera_factor;
				/*p.x = n;

				p.y = m;
*/
				p.x = (n - camera_cx)*p.z / camera_fx;

				p.y = (m - camera_cy)*p.z / camera_fy;

				//从rgb图像中获取它的颜色

				//rgb是三通道的BGR格式图，所以按下面的顺序获取颜色

				/*p.b = i_rgb.ptr<uchar>(m)[n * 3];

				p.g = i_rgb.ptr<uchar>(m)[n * 3 + 1];

				p.r = i_rgb.ptr<uchar>(m)[n * 3 + 2];*/

				//把p加入到点云中

				cloud->points.push_back(p);

			}

		//设置并保存点云

		cloud->height = 1;

		cloud->width = cloud->points.size();

		cout << "point cloud size=" << cloud->points.size() << endl;

		cloud->is_dense = false;

		//pcl::io::savePCDFile("C:/vsprojects/cvtest/cvtest/pointcloudyqy190509input.pcd", *cloud);
		pcl::PLYWriter writer;
		writer.write("C:/vsprojects/cvtest/cvtest/pointcloudyqy190511.ply", *cloud);

		//清楚数据并保存

		cloud->points.clear();

		cout << "Point cloud saved." << endl;





		if (27 == cv::waitKey(0))
			return 0;
	}
#endif


#if 0
	//获取rgb图和深度图成功，尝试解决深度图中墙面截断问题，存为16位图像成功，增加内参后点云也对了，现在存储的rgb图是已经和深度图对齐的rgb图
#include <kinect.h>
#include <iostream>
#include <opencv2\opencv.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <kinect.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include<string>


	//定义点云类型

	typedef pcl::PointXYZRGBA PointT;

	typedef pcl::PointCloud<PointT> PointCloud;

	////相机内参
	const double camera_factor = 500;

	const double camera_cx = 254.616;

	const double camera_cy = 207.801;

	const double camera_fx = 364.547;

	const double camera_fy = 364.547;

	//const double camera_cx = 259.896;

	//const double camera_cy = 206.745;

	//const double camera_fx = 367.749;

	//const double camera_fy = 367.749;


	using namespace cv;
	using namespace std;

	ICoordinateMapper* pMapper = nullptr;
	const int iWidth = 512, iHeight = 424;
	CameraSpacePoint depth2xyz[iWidth*iHeight];
	ColorSpacePoint depth2rgb[iWidth*iHeight];
	// 安全释放指针
	template<class Interface>
	inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}

	void getPointCloudFromImage(Mat depthImage, Mat rgbImage, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
	{

		pMapper->MapDepthFrameToCameraSpace(iWidth*iHeight, reinterpret_cast<UINT16*>(depthImage.data), iWidth*iHeight, depth2xyz);
		pMapper->MapDepthFrameToColorSpace(512 * 424, reinterpret_cast<UINT16*>(depthImage.data), 512 * 424, depth2rgb);

		cloud_out.height = 1;
		cloud_out.is_dense = 1;

		for (size_t i = 0; i < iWidth; i++)
		{
			for (size_t j = 0; j < iHeight; j++)
			{
				pcl::PointXYZRGB pointTemp;
				if (depth2xyz[i + j*iWidth].Z > 0.5)
				{

					pointTemp.x = depth2xyz[i + j*iWidth].X;
					pointTemp.y = depth2xyz[i + j*iWidth].Y;
					pointTemp.z = depth2xyz[i + j*iWidth].Z;
					int X = static_cast<int>(depth2rgb[j * 512 + i].X);
					int Y = static_cast<int>(depth2rgb[j * 512 + i].Y);
					if (X > 0 && Y > 0 && X < 1920 && Y < 1080)
					{
						Vec3b* pixelsRGBImage = rgbImage.ptr<Vec3b>(Y);
						pointTemp.g = pixelsRGBImage[X][0];
						pointTemp.b = pixelsRGBImage[X][1];
						pointTemp.r = pixelsRGBImage[X][2];
						cloud_out.push_back(pointTemp);
					}
					else continue;

				}
			}
		}
	}
	void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
	{

		viewer.setBackgroundColor(1, 1, 1);//设置背景颜色 


	}

	int countrgb = 1;
	int countdepth = 1;

	Mat ConvertMat_1(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
	{
		Mat img(nHeight, nWidth, CV_16UC1);
		UINT16* p_mat = (UINT16*)img.data;

		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		while (pBuffer < pBufferEnd)
		{
			//USHORT depth = *pBuffer;
			*p_mat = *pBuffer;
			//*p_mat = (depth >= nMinDepth) && (depth <= nMaxDepth) ? depth  : 0;
			//*p_mat = (depth >= nMinDepth) && (depth <= nMaxDepth) ? depth >> 0 : 0;
			p_mat++;
			++pBuffer;
		}
		return img;
	}


	int GetPicture()
	{


		// 获取Kinect设备
		IKinectSensor* m_pKinectSensor;
		HRESULT hr;
		hr = GetDefaultKinectSensor(&m_pKinectSensor);
		if (FAILED(hr))
		{
			return hr;
		}

		IMultiSourceFrameReader* m_pMultiFrameReader;
		if (m_pKinectSensor)
		{
			m_pKinectSensor->get_CoordinateMapper(&pMapper);
			hr = m_pKinectSensor->Open();
			if (SUCCEEDED(hr))
			{
				// 获取多数据源到读取器  
				hr = m_pKinectSensor->OpenMultiSourceFrameReader(
					FrameSourceTypes::FrameSourceTypes_Color |
					FrameSourceTypes::FrameSourceTypes_Infrared |
					FrameSourceTypes::FrameSourceTypes_Depth,
					&m_pMultiFrameReader);
			}
			

		}

		if (!m_pKinectSensor || FAILED(hr))
		{
			return E_FAIL;
		}
		// 三个数据帧及引用
		IDepthFrameReference* m_pDepthFrameReference;
		IColorFrameReference* m_pColorFrameReference;
		IInfraredFrameReference* m_pInfraredFrameReference;
		IInfraredFrame* m_pInfraredFrame;
		IDepthFrame* m_pDepthFrame;
		IColorFrame* m_pColorFrame;
		// 三个图片格式
		Mat i_rgb(1080, 1920, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
		Mat i_depth(424, 512, CV_16UC1);
		//Mat i_depth_32bir(424, 512, CV_32S);
		//Mat i_depth(424, 512, CV_16UC1);
		//Mat i_rgb(480,640, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
		//Mat i_depth(480, 640, CV_8UC1);
		Mat i_ir(424, 512, CV_16UC1);

		UINT16 *depthData = new UINT16[424 * 512];
		//INT32 *depthData_32bit = new INT32[424 * 512];
		IMultiSourceFrame* m_pMultiFrame = nullptr;
		int sample_id = 1;
		





		while (true)
		{
			// 获取新的一个多源数据帧

			hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
			if (FAILED(hr) || !m_pMultiFrame)
			{
				cout << "!!!" << endl;
				continue;
			}


			// 从多源数据帧中分离出彩色数据，深度数据和红外数据
			if (SUCCEEDED(hr))
				hr = m_pMultiFrame->get_ColorFrameReference(&m_pColorFrameReference);
			if (SUCCEEDED(hr))
				hr = m_pColorFrameReference->AcquireFrame(&m_pColorFrame);
			if (SUCCEEDED(hr))
				hr = m_pMultiFrame->get_DepthFrameReference(&m_pDepthFrameReference);
			if (SUCCEEDED(hr))
				hr = m_pDepthFrameReference->AcquireFrame(&m_pDepthFrame);
			if (SUCCEEDED(hr))
				hr = m_pMultiFrame->get_InfraredFrameReference(&m_pInfraredFrameReference);
			if (SUCCEEDED(hr))
				hr = m_pInfraredFrameReference->AcquireFrame(&m_pInfraredFrame);

			// color拷贝到图片中
			UINT nColorBufferSize = 1920 * 1080 * 4;
			//UINT nColorBufferSize = 640 * 480 * 4;
			if (SUCCEEDED(hr))
				hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);
			Mat i_depthToRgb(424, 512, CV_8UC4);
			// depth拷贝到图片中
			if (SUCCEEDED(hr))
			{
				//新增yqy
				hr = pMapper->MapDepthFrameToColorSpace(512 * 424, depthData, 512 * 424, depth2rgb);
				for (int i = 0; i < 424 * 512; i++)
				{
					ColorSpacePoint p = depth2rgb[i];
					if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
					{
						int colorX = static_cast<int>(p.X + 0.5f);
						int colorY = static_cast<int>(p.Y + 0.5f);

						if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
						{
							i_depthToRgb.data[i * 4] = i_rgb.data[(colorY * 1920 + colorX) * 4];
							i_depthToRgb.data[i * 4 + 1] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 1];
							i_depthToRgb.data[i * 4 + 2] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 2];
							i_depthToRgb.data[i * 4 + 3] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 3];
						}
					}
				}
				//yqyend
				USHORT nDepthMinReliableDistance = 0;//获取最大、最小深度距离信息
				USHORT nDepthMaxReliableDistance = 0;
				assert(hr >= 0);
				hr = m_pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
				assert(hr >= 0);
				hr = m_pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
		
				hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);

				i_depth=ConvertMat_1(depthData, 512, 424, nDepthMinReliableDistance, nDepthMaxReliableDistance);
				//hr = m_pDepthFrame->CopyFrameDataToArray(480 * 640, depthData);
				//for (int i = 0; i < 512 * 424; i++)
				//	//for (int i = 0; i < 640 * 480; i++)
				//{
				//	// 0-255深度图，为了显示明显，只取深度数据的低8位
				//	BYTE intensity = static_cast<BYTE>(depthData[i] % 65536);
				//	//BYTE intensity = static_cast<BYTE>(depthData[i] % 256);
				//	reinterpret_cast<BYTE*>(i_depth.data)[i] = intensity;
				//}

				// 实际是16位unsigned int数据
				//hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth.data));
			}

			// infrared拷贝到图片中
			if (SUCCEEDED(hr))
			{
				hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_ir.data));
				//hr = m_pInfraredFrame->CopyFrameDataToArray(480 * 640, reinterpret_cast<UINT16*>(i_ir.data));
			}


			//获取相机内参
			/*CameraIntrinsics* m_pCameraIntrinsics = new CameraIntrinsics();
			pMapper->GetDepthCameraIntrinsics(m_pCameraIntrinsics);
			cout << "FocalLengthX : " << m_pCameraIntrinsics->FocalLengthX << endl;
			cout << "FocalLengthY : " << m_pCameraIntrinsics->FocalLengthY << endl;
			cout << "PrincipalPointX : " << m_pCameraIntrinsics->PrincipalPointX << endl;
			cout << "PrincipalPointY : " << m_pCameraIntrinsics->PrincipalPointY << endl;*/




			// 显示
			imshow("rgb", i_depthToRgb);		
			std::stringstream str1;
			if (countrgb < 2)
			{
				str1 << "C:/vsprojects/test/test/result3/rgb/" << countrgb << ".png";
				countrgb++;
			}
			imwrite(str1.str(), i_depthToRgb);


			if (waitKey(1) == VK_ESCAPE)
				break;
			//cv::equalizeHist(i_depth, i_depth);//均衡化，为了提高显示效果
			imshow("depth", i_depth);
			//Mat img32BIT;
			////float scaleFactor = 1.0; // Or what you want 
			//i_depth.convertTo(img32BIT, CV_32S, 1.0 / 65535.0f);
	
			std::stringstream str2;
			if (countdepth <2)
			{
				str2 << "C:/vsprojects/test/test/result3/depth/" << countdepth << ".png";
				countdepth++;
			}
			imwrite(str2.str(), i_depth);
			//imwrite("C:/vsprojects/cvtest/cvtest/result/depth/yqydepth.png", i_depth);




			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			//遍历深度图

			for (int m = 0; m<i_depth.rows; m++)

				for (int n = 0; n<i_depth.cols; n++)

				{

					//获取深度图中(m,n)处的值

					ushort d = i_depth.ptr<ushort>(m)[n];

					//d可能没有值，若如此，跳过此点

					if (d == 0)

						continue;

					//d存在值，则向点云增加一个点

					//PointT p;
					pcl::PointXYZRGB p;

					//计算这个点的空间坐标

					//p.z = double(d); 
					p.z = double(d) / camera_factor;
					/*p.x = n ;

					p.y = m ;
*/
					p.x = (n - camera_cx)*p.z / camera_fx;

					p.y = (m - camera_cy)*p.z / camera_fy;

					//从rgb图像中获取它的颜色

					//rgb是三通道的BGR格式图，所以按下面的顺序获取颜色

					p.b = i_rgb.ptr<uchar>(m)[n * 3];

					p.g = i_rgb.ptr<uchar>(m)[n * 3 + 1];

					p.r = i_rgb.ptr<uchar>(m)[n * 3 + 2];

					//把p加入到点云中

					cloud->points.push_back(p);

				}

			//设置并保存点云

			cloud->height = 1;

			cloud->width = cloud->points.size();

			cout << "point cloud size=" << cloud->points.size() << endl;

			cloud->is_dense = false;

			//pcl::io::savePCDFile("C:/vsprojects/cvtest/cvtest/pointcloudyqy190509input.pcd", *cloud);
			pcl::PLYWriter writer;
			writer.write("C:/vsprojects/test/test/kinectcloud190519.ply", *cloud);

			//清楚数据并保存

			cloud->points.clear();

			cout << "Point cloud saved." << endl;







			if (waitKey(1) == VK_ESCAPE)
				break;
			imshow("ir", i_ir);
			if (waitKey(1) == VK_ESCAPE)
				break;
			/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
			getPointCloudFromImage(i_depth, i_rgb, *cloud_out);
			pcl::visualization::CloudViewer viewerG("Cloud Viewe");
			viewerG.runOnVisualizationThreadOnce(viewerOneOff);
			viewerG.showCloud(cloud_out);*/
			if (cv::waitKey(30) == VK_ESCAPE) break;
			//waitKey(0);
			/*string s1 = "C:\\Users\\mataiyuan\\Desktop\\yooongchun\\dataset\\sample-";
			string s2 = "-rgb.png";
			string s3 = "-depth.png";
			string s4 = "-infrared.png";
			cvSaveImage((s1 + to_string(sample_id) + s2).c_str(), &IplImage(i_rgb));
			cvSaveImage((s1 + to_string(sample_id) + s3).c_str(), &IplImage(i_depth));
			cvSaveImage((s1 + to_string(sample_id) + s4).c_str(), &IplImage(i_ir));
			sample_id += 1;*/

			// 释放资源
			SafeRelease(m_pColorFrame);
			SafeRelease(m_pDepthFrame);
			SafeRelease(m_pInfraredFrame);
			SafeRelease(m_pColorFrameReference);
			SafeRelease(m_pDepthFrameReference);
			SafeRelease(m_pInfraredFrameReference);
			SafeRelease(m_pMultiFrame);
		}
		// 关闭窗口，设备
		cv::destroyAllWindows();
		m_pKinectSensor->Close();
	}
	int main()
	{
		GetPicture();
		std::system("pause");
		return 0;
	}
#endif


#if 0
	//opengl测试成功
#include <GL/glut.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

	static int year = 0, spin = 0, day = 0;
	static GLint fogMode;
	const int n = 100;
	const GLfloat R = 1.0f;
	const GLfloat Pi = 3.1415926536f;

	void DrawCircle() {

		int  i;
		glClear(GL_COLOR_BUFFER_BIT);
		glBegin(GL_LINE_LOOP);

		for (i = 0; i < n; ++i)
		{
			glColor3f(1.0, 0.0, 0.0);
			glVertex2f(R*cos(2 * Pi / n*i), R*sin(2 * Pi / n*i));
		}

		glEnd();
		glFlush();
	}

	void init(void) {
		GLfloat position[] = { 0.5, 0.5, 3.0, 0.0 };
		glEnable(GL_DEPTH_TEST);                          //防止遮挡
		glLightfv(GL_LIGHT0, GL_POSITION, position);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);

		{
			GLfloat mat[3] = { 0.1745, 0.01175, 0.01175 };
			glMaterialfv(GL_FRONT, GL_AMBIENT, mat);
			mat[0] = 0.61424; mat[1] = 0.04136; mat[2] = 0.04136;
			glMaterialfv(GL_FRONT, GL_DIFFUSE, mat);
			mat[0] = 0.727811; mat[1] = 0.626959; mat[2] = 0.626959;
			glMaterialfv(GL_FRONT, GL_SPECULAR, mat);
			glMaterialf(GL_FRONT, GL_SHININESS, 0.6*128.0);
		}

		glEnable(GL_FOG);

		{
			GLfloat fogColor[4] = { 0.5, 0.5, 0.5, 1.0 };
			fogMode = GL_EXP;
			glFogi(GL_FOG_MODE, fogMode);
			glFogfv(GL_FOG_COLOR, fogColor);
			glFogf(GL_FOG_DENSITY, 0.35);
			glHint(GL_FOG_HINT, GL_DONT_CARE);
			glFogf(GL_FOG_START, 1.0);
			glFogf(GL_FOG_END, 5.0);
		}

		glClearColor(0.5, 0.9, 0.9, 1.0);  /* fog color */

	}

	void display(void) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glColor3f(0.0, 1.0, 1.0);
		glPushMatrix(); //记住自己的位置
		glutSolidSphere(1.0, 20, 16);   /* 画太阳半径、 20经度、16纬度*/
		glRotatef(spin, 0.0, 1.0, 0.0);  //自转，绕着一个向量以给定角度旋转（正的为逆时针）
		glTranslatef(2.0, 1.0, 0.0);
		glRotatef(spin, 1.0, 0.0, 0.0); //公转
		glRectf(0.1, 0.1, 0.5, 0.5);
		glColor3f(0.0, 0.0, 1.0);
		glutWireSphere(0.2, 8, 8);    /* 画第一颗小行星 */
		glColor3f(1.0, 0.0, 0.0);
		glTranslatef(2.0, 1.0, 0.0);
		glRotatef(2 * spin, 0.0, 1.0, 0.0);
		glutSolidSphere(0.5, 16, 8);
		glPopMatrix();//回到原来的位置
		glutSwapBuffers();
	}

	void spinDisplay(void) {
		spin = spin + 2;
		if (spin > 360)
			spin = spin - 360;
		glutPostRedisplay();
	}

	void mouse(int button, int state, int x, int y) {
		switch (button)
		{
		case GLUT_LEFT_BUTTON:
			if (state == GLUT_DOWN)
				glutIdleFunc(spinDisplay);
			break;

		case GLUT_MIDDLE_BUTTON:
			if (state == GLUT_DOWN)
				glutIdleFunc(NULL);
			break;

		default:
			break;
		}

	}

	void reshape(int w, int h) {
		glViewport(0, 0, (GLsizei)w, (GLsizei)h);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 0.5, 20.0);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(0.0, 10.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	}

	void keyboard(unsigned char key, int x, int y) {
		switch (key) {
		case 'd':
			day = (day + 10) % 360;
			glutPostRedisplay();
			break;
		case 'D':
			day = (day - 10) % 360;
			glutPostRedisplay();
			break;
		case 'y':
			year = (year + 5) % 360;
			glutPostRedisplay();
			break;
		case 'Y':
			year = (year - 5) % 360;
			glutPostRedisplay();
			break;
		case 27:
			exit(0);
			break;
		default:
			break;
		}
	}

	int main(int argc, char** argv) {
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
		glutInitWindowSize(400, 400);
		glutInitWindowPosition(100, 100);
		glutCreateWindow("OpengGL 程序设计");
		init();
		//glutDisplayFunc(DrawCircle);
		glutDisplayFunc(display);
		glutReshapeFunc(reshape);
		//glutKeyboardFunc(keyboard);
		glutMouseFunc(mouse);
		glutMainLoop();

		return 0;
	}
#endif