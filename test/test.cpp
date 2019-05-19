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
	Mat bgr(1080, 1920, CV_8UC3);
	bgr = imread("C:/vsprojects/test/test/result2/rgb/1.png");
	Mat depth(424, 512, CV_16UC1);
	depth = imread("C:/vsprojects/test/test/result2/depth/1.png", IMREAD_ANYDEPTH);   // ͼƬ�����ĸ�ʽ��һ���Ͷ���ʱ���һ����������������ĸ�ʽ����8UC3
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
	imwrite("registrationResult2.png", result);
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

//������

int main(int argc, char** argv)

{

	//��ȡ./data/rgb.png��./data/depth.png����ת��Ϊ����

	//ͼ�����

	cv::Mat rgb, depth;

	//ʹ��cv::imread()����ȡͼ��

	//rgbͼ����8UC3�Ĳ�ɫͼ��

	rgb = cv::imread("C:/vsprojects/test/test/inputimg190519/rgb/1.png");

	//depth��16UC1�ĵ�ͨ��ͼ��ע��flags����Ϊ-1����ʾ��ȡԭʼ���ݲ����޸�

	/*depth = cv::imread("C:/vsprojects/cvtest/cvtest/190509yqy/dt_1.png", -1);*/
	depth = cv::imread("C:/vsprojects/test/test/inputimg190519/depth/1.png", -1);
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
	writer.write("C:/vsprojects/test/test/kinectinput190519.ply", *cloud);

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




#if 1
//���ɵ��Ƴɹ�,���������ɫ������
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
 





#if 0
//һ�����Խ�����ͼǽ��ضϣ����ƴ洢�е�����
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
	// ת��depthͼ��cv::Mat
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
	// ת��colorͼ��cv::Mat
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
		int depth_width = 512; //depthͼ�������ôС
		int depth_height = 424;
		int color_widht = 1920; //colorͼ�������ô��
		int color_height = 1080;

		//cv::Mat depthImg_show = cv::Mat::zeros(depth_height, depth_width, CV_8UC3);//ԭʼUINT16 ���ͼ���ʺ�������ʾ��������Ҫ����8λ�ľͿ����ˣ�������ʾ����Ҳ���Ƿǳ��ã��������ԭʼ16λͼ����ɫ���룬�պ��ſ���
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

		while (true) // ò��Ҫһֱ���ԣ���һ��ÿ�ζ��ܶ�ȡ��ͼ��
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
					if (imageFormat == ColorImageFormat_Bgra)//����������format����֪�����庬�壬���һ��Ԥ�ȷ����ڴ棬һ����Ҫ�Լ����ռ��
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
				//�������ͼ

				for (int m = 0; m<depthImg_show.rows; m++)

					for (int n = 0; n<depthImg_show.cols; n++)

					{

						//��ȡ���ͼ��(m,n)����ֵ

						ushort d = depthImg_show.ptr<ushort>(m)[n];

						//d����û��ֵ������ˣ������˵�

						if (d == 0)

							continue;

						//d����ֵ�������������һ����

						//PointT p;
						pcl::PointXYZRGB p;

						//���������Ŀռ�����

						//p.z = double(d); 
						p.z = double(d) / 500;
						p.x = n;

						p.y = m;

						/*p.x = (n - camera_cx)*p.z / camera_fx;

						p.y = (m - camera_cy)*p.z / camera_fy;*/

						//��rgbͼ���л�ȡ������ɫ

						//rgb����ͨ����BGR��ʽͼ�����԰������˳���ȡ��ɫ

						/*p.b = i_rgb.ptr<uchar>(m)[n * 3];

						p.g = i_rgb.ptr<uchar>(m)[n * 3 + 1];

						p.r = i_rgb.ptr<uchar>(m)[n * 3 + 2];*/

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
	//���Խ�����ͼ��ǽ��ض����⣬��Ϊ16λͼ��ɹ��������ڲκ����Ҳ����
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
//�����������

	typedef pcl::PointXYZRGBA PointT;

	typedef pcl::PointCloud<PointT> PointCloud;

	////����ڲ�
	const double camera_factor = 500;

	const double camera_cx = 256;

	const double camera_cy = 212;

	const double camera_fx = 367.749;

	const double camera_fy = 367.749;
	using namespace cv;

	// ת��depthͼ��cv::Mat
	Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight)
	{


		cv::Mat img(nHeight, nWidth, CV_16UC1);
		UINT16* p_mat = (UINT16*)img.data;//yqy �ص㣡������������uchar����UINT16

		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		while (pBuffer < pBufferEnd)
		{
			*p_mat = *pBuffer;
			p_mat++;
			++pBuffer;
		}



		//Mat img(nHeight, nWidth, CV_8UC1);
		//uchar* p_mat = img.data;//ָ��ͷָ��

		//const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);//ָ�����һ��Ԫ�ص�ָ��

		//while (pBuffer < pBufferEnd)//16λ���ֵΪ65536
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

		HRESULT hr = GetDefaultKinectSensor(&m_pKinectSensor);//��ȡĬ��kinect������
		assert(hr >= 0);
		printf("��kinect�������ɹ�\n");

		hr = m_pKinectSensor->Open();//�򿪴�����
		assert(hr >= 0);
		hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);//��������Ϣ������
		assert(hr >= 0);
		hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);//�������Ϣ֡��ȡ��
		assert(hr >= 0);

		while (hr < 0 || pDepthFrame == NULL)
			hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//������ʱ���ȡ���������ѭ����ȡ�����֡

		assert(hr >= 0);
		hr = pDepthFrame->get_FrameDescription(&pFrameDescription);//��ȡ֡��������Ϣ����͸ߣ�
		int depth_width, depth_height;
		pFrameDescription->get_Width(&depth_width);
		pFrameDescription->get_Height(&depth_height);
		printf("width=%d height=%d\n", depth_width, depth_height);

		USHORT nDepthMinReliableDistance = 0;//��ȡ�����С��Ⱦ�����Ϣ
		USHORT nDepthMaxReliableDistance = 0;
		assert(hr >= 0);
		hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
		assert(hr >= 0);
		hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);

		printf("nDepthMinReliableDistance=%d nDepthMaxReliableDistance=%d\n", nDepthMinReliableDistance, nDepthMaxReliableDistance);

		UINT nBufferSize_depth = 0;
		UINT16 *pBuffer_depth = NULL;
		pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);//��ȡͼ�����ظ�����ָ��ͼ���ָ��


																				//ת��ΪMAT��ʽ
		Mat i_depth = ConvertMat(pBuffer_depth, depth_width, depth_height);//ת��Ϊ8λ��mat


		//cv::equalizeHist(i_depth, i_depth);//���⻯��Ϊ�������ʾЧ��

		cv::imwrite("MyFirstKinectImg.png", i_depth);//����ͼƬ
													   //��opencv��ʾ

		cv::namedWindow("display");

		cv::imshow("display", i_depth);




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

				//p.z = double(d); 
				p.z = double(d) / camera_factor;
				/*p.x = n;

				p.y = m;
*/
				p.x = (n - camera_cx)*p.z / camera_fx;

				p.y = (m - camera_cy)*p.z / camera_fy;

				//��rgbͼ���л�ȡ������ɫ

				//rgb����ͨ����BGR��ʽͼ�����԰������˳���ȡ��ɫ

				/*p.b = i_rgb.ptr<uchar>(m)[n * 3];

				p.g = i_rgb.ptr<uchar>(m)[n * 3 + 1];

				p.r = i_rgb.ptr<uchar>(m)[n * 3 + 2];*/

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





		if (27 == cv::waitKey(0))
			return 0;
	}
#endif


#if 0
	//��ȡrgbͼ�����ͼ�ɹ������Խ�����ͼ��ǽ��ض����⣬��Ϊ16λͼ��ɹ��������ڲκ����Ҳ���ˣ����ڴ洢��rgbͼ���Ѿ������ͼ�����rgbͼ
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
		Mat i_depth(424, 512, CV_16UC1);
		//Mat i_depth_32bir(424, 512, CV_32S);
		//Mat i_depth(424, 512, CV_16UC1);
		//Mat i_rgb(480,640, CV_8UC4);      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
		//Mat i_depth(480, 640, CV_8UC1);
		Mat i_ir(424, 512, CV_16UC1);

		UINT16 *depthData = new UINT16[424 * 512];
		//INT32 *depthData_32bit = new INT32[424 * 512];
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
			Mat i_depthToRgb(424, 512, CV_8UC4);
			// depth������ͼƬ��
			if (SUCCEEDED(hr))
			{
				//����yqy
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
				USHORT nDepthMinReliableDistance = 0;//��ȡ�����С��Ⱦ�����Ϣ
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
				//	// 0-255���ͼ��Ϊ����ʾ���ԣ�ֻȡ������ݵĵ�8λ
				//	BYTE intensity = static_cast<BYTE>(depthData[i] % 65536);
				//	//BYTE intensity = static_cast<BYTE>(depthData[i] % 256);
				//	reinterpret_cast<BYTE*>(i_depth.data)[i] = intensity;
				//}

				// ʵ����16λunsigned int����
				//hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth.data));
			}

			// infrared������ͼƬ��
			if (SUCCEEDED(hr))
			{
				hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_ir.data));
				//hr = m_pInfraredFrame->CopyFrameDataToArray(480 * 640, reinterpret_cast<UINT16*>(i_ir.data));
			}


			//��ȡ����ڲ�
			/*CameraIntrinsics* m_pCameraIntrinsics = new CameraIntrinsics();
			pMapper->GetDepthCameraIntrinsics(m_pCameraIntrinsics);
			cout << "FocalLengthX : " << m_pCameraIntrinsics->FocalLengthX << endl;
			cout << "FocalLengthY : " << m_pCameraIntrinsics->FocalLengthY << endl;
			cout << "PrincipalPointX : " << m_pCameraIntrinsics->PrincipalPointX << endl;
			cout << "PrincipalPointY : " << m_pCameraIntrinsics->PrincipalPointY << endl;*/




			// ��ʾ
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
			//cv::equalizeHist(i_depth, i_depth);//���⻯��Ϊ�������ʾЧ��
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

					//p.z = double(d); 
					p.z = double(d) / camera_factor;
					/*p.x = n ;

					p.y = m ;
*/
					p.x = (n - camera_cx)*p.z / camera_fx;

					p.y = (m - camera_cy)*p.z / camera_fy;

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
			writer.write("C:/vsprojects/test/test/kinectcloud190519.ply", *cloud);

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