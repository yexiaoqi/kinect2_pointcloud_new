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
	//depth = cv::imread("C:/vsprojects/test/test/inputimg190519/depth/1.png", -1);
	depth = cv::imread("C:/vsprojects/kinect_depth_inpainting_and_filtering/kinect_depth_inpainting_and_filtering/inpaint.png", -1);
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
	writer.write("C:/vsprojects/test/test/16bitonlyinpaint190530.ply", *cloud);

	//������ݲ�����

	cloud->points.clear();

	cout << "Point cloud saved." << endl;

	return 0;

}
#endif


#if 0
//8λ���ͼת����
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
	//depth = cv::imread("C:/vsprojects/test/test/inputimg190519/depth/1.png", -1);
	/*depth = cv::imread("C:/vsprojects/test/test/result3/depthinpaint/2.png",-1);*/
	depth = cv::imread("C:/vsprojects/kinect_depth_inpainting_and_filtering/kinect_depth_inpainting_and_filtering/filtered median.png", -1);

	//rgb = cv::imread("C:/vsprojects/cvtest/cvtest/20170907/group1/color_map/frame_000001.png");

	////depth��16UC1�ĵ�ͨ��ͼ��ע��flags����Ϊ-1����ʾ��ȡԭʼ���ݲ����޸�

	//depth = cv::imread("C:/vsprojects/cvtest/cvtest/20170907/group1/depth_map/frame_000001.png", -1);

	//���Ʊ���

	//ʹ������ָ�룬����һ���յ��ơ�����ָ��������Զ��ͷ�

	//PointCloud::Ptr cloud(new PointCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//�������ͼ

	for (int m = 0; m < depth.rows; m++)

		for (int n = 0; n < depth.cols; n++)

		{

			//��ȡ���ͼ��(m,n)����ֵ

			//ushort d = depth.ptr<ushort>(m)[n];
			uchar d = depth.ptr<uchar>(m)[n];//8λҪ��Ϊuchar����������������������������������������������������������������������������

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
			//p.z = double(d)/ camera_factor;

			//p.x = n;

			//p.y = m ;

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
	writer.write("C:/vsprojects/test/test/onlymidfilter190529.ply", *cloud);

	//������ݲ�����

	cloud->points.clear();

	cout << "Point cloud saved." << endl;

	return 0;
}
#endif



#if 0
//���ͼ��rgbͼת��Ϊ����,��Ϊpcd��ʽ
#include<iostream>

#include<string>

using namespace std;

//opencv��
#include <opencv2/opencv.hpp>
//#include<opencv2/core/core.cpp>

//#include<opencv2/highgui/highgui.hpp>

//PCL��

#include<pcl/io/pcd_io.h>

#include<pcl/point_types.h>






//�����������

typedef pcl::PointXYZRGBA PointT;

typedef pcl::PointCloud<PointT> PointCloud;

//����ڲ�
//
const double camera_factor = 500;

const double camera_cx = 254.616;

const double camera_cy = 207.801;

const double camera_fx = 364.547;

const double camera_fy = 364.547;
//const double camera_factor = 1000;
//
//const double camera_cx = 259.896;
//
//const double camera_cy = 206.745;
//
//const double camera_fx = 367.749;
//
//const double camera_fy = 367.749;

//������

int main(int argc, char** argv)

{

	//��ȡ./data/rgb.png��./data/depth.png����ת��Ϊ����

	//ͼ�����

	cv::Mat rgb, depth;

	//ʹ��cv::imread()����ȡͼ��

	//rgbͼ����8UC3�Ĳ�ɫͼ��

	rgb = cv::imread("C:/vsprojects/test/test/result/rgb2/1.png");

	//depth��16UC1�ĵ�ͨ��ͼ��ע��flags����Ϊ-1����ʾ��ȡԭʼ���ݲ����޸�

	/*depth = cv::imread("C:/vsprojects/cvtest/cvtest/190509yqy/dt_1.png", -1);*/
	depth = cv::imread("C:/vsprojects/test/test/result/depth/1.png", -1);
	//rgb = cv::imread("C:/vsprojects/cvtest/cvtest/20170907/group1/color_map/frame_000001.png");

	////depth��16UC1�ĵ�ͨ��ͼ��ע��flags����Ϊ-1����ʾ��ȡԭʼ���ݲ����޸�

	//depth = cv::imread("C:/vsprojects/cvtest/cvtest/20170907/group1/depth_map/frame_000001.png", -1);

	//���Ʊ���

	//ʹ������ָ�룬����һ���յ��ơ�����ָ��������Զ��ͷ�

	PointCloud::Ptr cloud(new PointCloud);

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

			PointT p;

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

	pcl::io::savePCDFile("C:/vsprojects/test/test/pointcloud/forunsample190603.pcd", *cloud);

	//������ݲ�����

	cloud->points.clear();

	cout << "Point cloud saved." << endl;

	return 0;

}
#endif





//��ԺͲ�ɫͼ�����1920*1080���ͼ������ת��Ϊ���ƣ���ʹ���ڲ�ֱ��ӳ��
#if 1
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
//const double camera_cx = 965.49803;
//
//const double camera_cy = 526.73740;
//
//const double camera_fx = 1096.03541;
//
//const double camera_fy = 1103.58516;
////const double camera_factor = 1000;





const double camera_factor = 1000;
//
//const double camera_cx = 976.3789;
//
//const double camera_cy = 1003.8;
//
//const double camera_fx = 983.3159;
//
//const double camera_fy = 623.4680;


const double camera_cx = 1065.5;

const double camera_cy = 1071.2;

const double camera_fx = 974.7039;

const double camera_fy = 542.3027;
;



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

	rgb = cv::imread("C:/vsprojects/test/test/result190604/rgb19201080/19.png");

	//depth��16UC1�ĵ�ͨ��ͼ��ע��flags����Ϊ-1����ʾ��ȡԭʼ���ݲ����޸�

	/*depth = cv::imread("C:/vsprojects/cvtest/cvtest/190509yqy/dt_1.png", -1);*/
	//depth = cv::imread("C:/vsprojects/test/test/imgresult/2.png", -1);
	depth = cv::imread("C:/vsprojects/Robust-Color-Guided-Depth-Map-Restoration-master/Robust-Color-Guided-Depth-Map-Restoration-master/yqy/2inpaintopencv.png", -1);
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
			p.z = double(d) ;

			p.x = n ;

			p.y = m;

			/*p.z = double(d) / camera_factor;

			p.x = (n - camera_cx)*p.z / camera_fx;

			p.y = (m - camera_cy)*p.z / camera_fy;*/

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
	writer.write("C:/vsprojects/test/test/result190604/190606172inpaintopencv.ply", *cloud);

	//������ݲ�����

	cloud->points.clear();

	cout << "Point cloud saved." << endl;

	return 0;

}
#endif