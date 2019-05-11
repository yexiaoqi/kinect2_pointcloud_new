#include "stdafx.h"



#if 0

/*
�����ͼӳ�䵽��ɫͼ�ϣ����ɺ����ͼƥ��Ķ����ɫͼ,Ч�����Ǻܺ������Ȳ�����
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
	// ���ز���
	ifstream parm("registration.txt");
	string stringLine;
	if (parm.is_open())
	{
		// rgb���������fx,fy,cx,cy
		getline(parm, stringLine);
		stringstream lin(stringLine);
		string s1, s2, s3, s4, s5, s6, s7, s8, s9;
		lin >> s1 >> s2 >> s3 >> s4;
		kinectParm->fx_rgb = atof(s1.c_str());
		kinectParm->fy_rgb = atof(s2.c_str());
		kinectParm->cx_rgb = atof(s3.c_str());
		kinectParm->cy_rgb = atof(s4.c_str());
		stringLine.clear();
		// ir���������fx,fy,cx,cy
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
	// 1. ��ȡ����
	KinectParm *parm = new KinectParm();
	if (!loadParm(parm))
		return 0;
	// 2. ����rgbͼƬ�����ͼ����ʾ
	Mat bgr(1080, 1920, CV_8UC4);
	bgr = imread("C:/vsprojects/cvtest/cvtest/result/rgb/1.png");
	Mat depth(424, 512, CV_16UC1);
	depth = imread("C:/vsprojects/cvtest/cvtest/result/depth/1.png", IMREAD_ANYDEPTH);   // ͼƬ�����ĸ�ʽ��һ���Ͷ���ʱ���һ����������������ĸ�ʽ����8UC3
	Mat depth2rgb = imread("depth2rgb.jpg");
	// 3. ��ʾ
	thread th = std::thread([&] {
		while (true)
		{
			imshow("ԭʼ��ɫͼ", bgr);
			waitKey(1);
			imshow("ԭʼ���ͼ", depth);
			waitKey(1);
			imshow("ԭʼͶӰͼ", depth2rgb);
			waitKey(1);
		}
	});
	// 4. �任

	// 4.1 �����������
#pragma region  �����
	Eigen::Matrix3f K_ir;           // ir�ڲξ���
	K_ir <<
		parm->fx_ir, 0, parm->cx_ir,
		0, parm->fy_ir, parm->cy_ir,
		0, 0, 1;
	Eigen::Matrix3f K_rgb;          // rgb�ڲξ���
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


	// 4.2 ����ͶӰ
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
				// ͶӰ����ɫͼ�ϵ�����
				Eigen::Vector3f uv_depth(col, row, 1.0f);                            // !!!p_ir
				Eigen::Vector3f uv_color = depthValue / 1000.f*R*uv_depth + T / 1000;   // !!!Z_rgb*p_rgb=R*Z_ir*p_ir+T; (����1000����Ϊ�˴Ӻ��ױ���)

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
	imwrite("registrationResult.png", result);
	thread th2 = std::thread([&] {
		while (true)
		{
			imshow("���ͼ", result);
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
//���ͼ��rgbͼת��Ϊ����,���ڸĳ���ֱ�Ӵ�Ϊply������pcd��ʽ
#include<iostream>

#include<string>

using namespace std;

//opencv��
#include <opencv2/opencv.hpp>
//#include<opencv2/core/core.cpp>

//#include<opencv2/highgui/highgui.hpp>

//PCL��

#include<pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include<pcl/point_types.h>
#include <pcl/point_cloud.h>





//�����������

typedef pcl::PointXYZRGBA PointT;

typedef pcl::PointCloud<PointT> PointCloud;

//����ڲ�
////
//const double camera_factor = 1000;
//
//const double camera_cx = 325.5;
//
//const double camera_cy = 253.5;
//
//const double camera_fx = 518.0;
//
//const double camera_fy = 519.0;
const double camera_factor = 1000;

const double camera_cx = 200;

const double camera_cy = 200;
//const double camera_fx = 1000;
//
//const double camera_fy = 1000;

const double camera_fx = 367.749;

const double camera_fy = 367.749;

//������

int main(int argc, char** argv)

{

	//��ȡ./data/rgb.png��./data/depth.png����ת��Ϊ����

	//ͼ�����

	cv::Mat rgb, depth;

	//ʹ��cv::imread()����ȡͼ��

	//rgbͼ����8UC3�Ĳ�ɫͼ��

	rgb = cv::imread("C:/vsprojects/cvtest/cvtest/result/rgb2/1.png");

	//depth��16UC1�ĵ�ͨ��ͼ��ע��flags����Ϊ-1����ʾ��ȡԭʼ���ݲ����޸�

	/*depth = cv::imread("C:/vsprojects/cvtest/cvtest/190509yqy/dt_1.png", -1);*/
	depth = cv::imread("C:/vsprojects/cvtest/cvtest/result/depth/1.png", -1);
	//rgb = cv::imread("C:/vsprojects/cvtest/cvtest/20170907/group1/color_map/frame_000001.png");

	////depth��16UC1�ĵ�ͨ��ͼ��ע��flags����Ϊ-1����ʾ��ȡԭʼ���ݲ����޸�

	//depth = cv::imread("C:/vsprojects/cvtest/cvtest/20170907/group1/depth_map/frame_000001.png", -1);

	//���Ʊ���

	//ʹ������ָ�룬����һ���յ��ơ�����ָ��������Զ��ͷ�

	//PointCloud::Ptr cloud(new PointCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//�������ͼ

	for (int m = 0; m<depth.rows; m++)

		for (int n = 0; n<depth.cols; n++)

		{

			//��ȡ���ͼ��(m,n)����ֵ

			ushort d = depth.ptr<ushort>(m)[n];

			//d����û��ֵ������ˣ������˵�

			if (d == 0)

				continue;

			//d����ֵ�������������һ����

			//PointT p;
			pcl::PointXYZRGB p;

			//���������Ŀռ�����

			p.z = double(d) / camera_factor;

			p.x = (n - camera_cx)*p.z / camera_fx;

			p.y = (m - camera_cy)*p.z / camera_fy;

			//��rgbͼ���л�ȡ������ɫ

			//rgb����ͨ����BGR��ʽͼ�����԰������˳���ȡ��ɫ

			p.b = rgb.ptr<uchar>(m)[n * 3];

			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];

			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

			//��p���뵽������

			cloud->points.push_back(p);

		}

	//���ò��������

	cloud->height = 1;

	cloud->width = cloud->points.size();

	cout << "point cloud size=" << cloud->points.size() << endl;

	cloud->is_dense = false;

	//pcl::io::savePCDFile("C:/vsprojects/cvtest/cvtest/pointcloudyqy190509input.pcd", *cloud);
	pcl::PLYWriter writer;
	writer.write("C:/vsprojects/cvtest/cvtest/test..ply", *cloud);

	//������ݲ�����

	cloud->points.clear();

	cout << "Point cloud saved." << endl;

	return 0;

}
#endif



#if 0
//pcdתply�ļ��ɹ�
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
�����ͼӳ�䵽��ɫͼ�ϣ����ɺ����ͼƥ��Ķ����ɫͼ,Ч�����Ǻܺ������Ȳ�����
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
	// ���ز���
	ifstream parm("registration.txt");
	string stringLine;
	if (parm.is_open())
	{
		// rgb���������fx,fy,cx,cy
		getline(parm, stringLine);
		stringstream lin(stringLine);
		string s1, s2, s3, s4, s5, s6, s7, s8, s9;
		lin >> s1 >> s2 >> s3 >> s4;
		kinectParm->fx_rgb = atof(s1.c_str());
		kinectParm->fy_rgb = atof(s2.c_str());
		kinectParm->cx_rgb = atof(s3.c_str());
		kinectParm->cy_rgb = atof(s4.c_str());
		stringLine.clear();
		// ir���������fx,fy,cx,cy
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


	// 1. ��ȡ����
	KinectParm *parm = new KinectParm();
	if (!loadParm(parm))
		return 0;
	// 4.1 �����������
	//#pragma region  �����
	Eigen::Matrix3f K_ir;           // ir�ڲξ���
	K_ir <<
		parm->fx_ir, 0, parm->cx_ir,
		0, parm->fy_ir, parm->cy_ir,
		0, 0, 1;
	Eigen::Matrix3f K_rgb;          // rgb�ڲξ���
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
		
		// 2. ����rgbͼƬ�����ͼ����ʾ
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
		depth = imread("C:/vsprojects/cvtest/cvtest/result/depth/1.png", IMREAD_ANYDEPTH);   // ͼƬ�����ĸ�ʽ��һ���Ͷ���ʱ���һ����������������ĸ�ʽ����8UC3
		Mat depth2rgb = imread("depth2rgb.jpg");
		// 3. ��ʾ
		thread th = std::thread([&] {
			while (true)
			{
				imshow("ԭʼ��ɫͼ", bgr);
				waitKey(1);
				imshow("ԭʼ���ͼ", depth);
				waitKey(1);
				imshow("ԭʼͶӰͼ", depth2rgb);
				waitKey(1);
			}
		});
		// 4. �任
		cout << depth.type() << endl;
		

		// 4.2 ����ͶӰ
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
					// ͶӰ����ɫͼ�ϵ�����
					Eigen::Vector3f uv_depth(col, row, 1.0f);                            // !!!p_ir
					Eigen::Vector3f uv_color = depthValue / 1000.f*R*uv_depth + T / 1000;   // !!!Z_rgb*p_rgb=R*Z_ir*p_ir+T; (����1000����Ϊ�˴Ӻ��ױ���)

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
				imshow("���ͼ", result);
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




#if 0
//���ɵ��Ƴɹ�
#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointType;

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

	while (!viewer->wasStopped()) {
		// Update Viewer
		viewer->spinOnce();

		boost::mutex::scoped_try_lock lock(mutex);
		pcl::PLYWriter writer;
		writer.write("yqyou2.ply", *cloud);
		if (lock.owns_lock() && cloud) {
			// Update Point Cloud
			if (!viewer->updatePointCloud(cloud, "cloud")) {
				viewer->addPointCloud(cloud, "cloud");

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
//���ɵ���ʧ��
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

	viewer.setBackgroundColor(1, 1, 1);//���ñ�����ɫ 


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
	// ����ͼƬ��ʽ
	Mat i_rgb(1080, 1920, CV_8UC4);      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
	Mat i_depth(424, 512, CV_8UC1);
	//Mat i_rgb(480,640, CV_8UC4);      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
	//Mat i_depth(480, 640, CV_8UC1);
	Mat i_ir(424, 512, CV_16UC1);

	UINT16 *depthData = new UINT16[424 * 512];
	IMultiSourceFrame* m_pMultiFrame = nullptr;
	int sample_id = 1;
	while (true)
	{
		// ��ȡ�µ�һ����Դ����֡

		hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
		if (FAILED(hr) || !m_pMultiFrame)
		{
			cout << "!!!" << endl;
			continue;
		}


		// �Ӷ�Դ����֡�з������ɫ���ݣ�������ݺͺ�������
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
		//pcl::io::savePCDFileASCII("�������.pcd", *cloud_out);
		pcl::PLYWriter writer;
		writer.write("out190509.ply", *cloud_out);
		viewerG.runOnVisualizationThreadOnce(viewerOneOff);
		viewerG.showCloud(cloud_out);
		while (true) if (cv::waitKey(30) == VK_ESCAPE) break;
		return 0;
	}
#endif


#if 1
	//��ȡrgbͼ�����ͼ�ɹ�
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


	//�����������

	typedef pcl::PointXYZRGBA PointT;

	typedef pcl::PointCloud<PointT> PointCloud;

	////����ڲ�
	const double camera_factor = 1000;

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
	// ��ȫ�ͷ�ָ��
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

		viewer.setBackgroundColor(1, 1, 1);//���ñ�����ɫ 


	}

	int countrgb = 1;
	int countdepth = 1;

	int GetPicture()
	{
		// ��ȡKinect�豸
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
				// ��ȡ������Դ����ȡ��  
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
		// ��������֡������
		IDepthFrameReference* m_pDepthFrameReference;
		IColorFrameReference* m_pColorFrameReference;
		IInfraredFrameReference* m_pInfraredFrameReference;
		IInfraredFrame* m_pInfraredFrame;
		IDepthFrame* m_pDepthFrame;
		IColorFrame* m_pColorFrame;
		// ����ͼƬ��ʽ
		Mat i_rgb(1080, 1920, CV_8UC4);      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
		Mat i_depth(424, 512, CV_8UC1);
		//Mat i_depth(424, 512, CV_16UC1);
		//Mat i_rgb(480,640, CV_8UC4);      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
		//Mat i_depth(480, 640, CV_8UC1);
		Mat i_ir(424, 512, CV_16UC1);

		UINT16 *depthData = new UINT16[424 * 512];
		IMultiSourceFrame* m_pMultiFrame = nullptr;
		int sample_id = 1;
		
		while (true)
		{
			// ��ȡ�µ�һ����Դ����֡

			hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
			if (FAILED(hr) || !m_pMultiFrame)
			{
				cout << "!!!" << endl;
				continue;
			}


			// �Ӷ�Դ����֡�з������ɫ���ݣ�������ݺͺ�������
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

			// color������ͼƬ��
			UINT nColorBufferSize = 1920 * 1080 * 4;
			//UINT nColorBufferSize = 640 * 480 * 4;
			if (SUCCEEDED(hr))
				hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);

			// depth������ͼƬ��
			if (SUCCEEDED(hr))
			{
				hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);
				//hr = m_pDepthFrame->CopyFrameDataToArray(480 * 640, depthData);
				for (int i = 0; i < 512 * 424; i++)
					//for (int i = 0; i < 640 * 480; i++)
				{
					// 0-255���ͼ��Ϊ����ʾ���ԣ�ֻȡ������ݵĵ�8λ
					//BYTE intensity = static_cast<BYTE>(depthData[i] % 65536);
					BYTE intensity = static_cast<BYTE>(depthData[i] % 256);
					reinterpret_cast<BYTE*>(i_depth.data)[i] = intensity;
				}

				// ʵ����16λunsigned int����
				//hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth.data));
			}

			// infrared������ͼƬ��
			if (SUCCEEDED(hr))
			{
				hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_ir.data));
				//hr = m_pInfraredFrame->CopyFrameDataToArray(480 * 640, reinterpret_cast<UINT16*>(i_ir.data));
			}

			// ��ʾ
			imshow("rgb", i_rgb);
			std::stringstream str1;
			if (countrgb < 2)
			{
				str1 << "C:/vsprojects/test/test/result/rgb/" << countrgb << ".png";
				countrgb++;
			}
			imwrite(str1.str(), i_rgb);


			if (waitKey(1) == VK_ESCAPE)
				break;
			imshow("depth", i_depth);
			std::stringstream str2;
			if (countdepth < 2)
			{
				str2 << "C:/vsprojects/test/test/result/depth/" << countdepth << ".png";
				countdepth++;
			}
			imwrite(str2.str(), i_depth);
			//imwrite("C:/vsprojects/cvtest/cvtest/result/depth/yqydepth.png", i_depth);




			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			//�������ͼ

			for (int m = 0; m<i_depth.rows; m++)

				for (int n = 0; n<i_depth.cols; n++)

				{

					//��ȡ���ͼ��(m,n)����ֵ

					ushort d = i_depth.ptr<ushort>(m)[n];

					//d����û��ֵ������ˣ������˵�

					if (d == 0)

						continue;

					//d����ֵ�������������һ����

					//PointT p;
					pcl::PointXYZRGB p;

					//���������Ŀռ�����


					p.z = double(d) / camera_factor;
					p.x = n ;

					p.y = m ;

					/*p.x = (n - camera_cx)*p.z / camera_fx;

					p.y = (m - camera_cy)*p.z / camera_fy;*/

					//��rgbͼ���л�ȡ������ɫ

					//rgb����ͨ����BGR��ʽͼ�����԰������˳���ȡ��ɫ

					p.b = i_rgb.ptr<uchar>(m)[n * 3];

					p.g = i_rgb.ptr<uchar>(m)[n * 3 + 1];

					p.r = i_rgb.ptr<uchar>(m)[n * 3 + 2];

					//��p���뵽������

					cloud->points.push_back(p);

				}

			//���ò��������

			cloud->height = 1;

			cloud->width = cloud->points.size();

			cout << "point cloud size=" << cloud->points.size() << endl;

			cloud->is_dense = false;

			//pcl::io::savePCDFile("C:/vsprojects/cvtest/cvtest/pointcloudyqy190509input.pcd", *cloud);
			pcl::PLYWriter writer;
			writer.write("C:/vsprojects/cvtest/cvtest/pointcloudyqy190511.ply", *cloud);

			//������ݲ�����

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

			// �ͷ���Դ
			SafeRelease(m_pColorFrame);
			SafeRelease(m_pDepthFrame);
			SafeRelease(m_pInfraredFrame);
			SafeRelease(m_pColorFrameReference);
			SafeRelease(m_pDepthFrameReference);
			SafeRelease(m_pInfraredFrameReference);
			SafeRelease(m_pMultiFrame);
		}
		// �رմ��ڣ��豸
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
	//opengl���Գɹ�
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
		glEnable(GL_DEPTH_TEST);                          //��ֹ�ڵ�
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
		glPushMatrix(); //��ס�Լ���λ��
		glutSolidSphere(1.0, 20, 16);   /* ��̫���뾶�� 20���ȡ�16γ��*/
		glRotatef(spin, 0.0, 1.0, 0.0);  //��ת������һ�������Ը����Ƕ���ת������Ϊ��ʱ�룩
		glTranslatef(2.0, 1.0, 0.0);
		glRotatef(spin, 1.0, 0.0, 0.0); //��ת
		glRectf(0.1, 0.1, 0.5, 0.5);
		glColor3f(0.0, 0.0, 1.0);
		glutWireSphere(0.2, 8, 8);    /* ����һ��С���� */
		glColor3f(1.0, 0.0, 0.0);
		glTranslatef(2.0, 1.0, 0.0);
		glRotatef(2 * spin, 0.0, 1.0, 0.0);
		glutSolidSphere(0.5, 16, 8);
		glPopMatrix();//�ص�ԭ����λ��
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
		glutCreateWindow("OpengGL �������");
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