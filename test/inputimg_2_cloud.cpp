#include "stdafx.h"
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
