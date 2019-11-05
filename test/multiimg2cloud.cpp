#include "stdafx.h"
#if 1
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
const double camera_factor = 1000;//米到毫米，1000！！！！！！！
//
//const double camera_cx = 254.616;
//
//const double camera_cy = 207.801;
//
//const double camera_fx = 364.547;
//
//const double camera_fy = 364.547;


const double camera_cx = 253.0139;

const double camera_cy = 209.5703;

const double camera_fx = 363.4980;

const double camera_fy = 365.6199;

//主函数


vector<double> worldT1 = { -0.0244328 ,0.14467 ,-1.51547 };
vector<vector<double>> worldR1 = { { 0.987309, 0.0770974, 0.13884 },{ 0.158686 ,- 0.444535 ,- 0.881593 },{ -0.00624941 ,0.892437 ,- 0.451128 } };
vector<double> worldT2 = { 0.11668 ,-0.0769234 ,-2.01791 };
vector<vector<double>> worldR2 = { { -0.969907 ,- 0.0435608 ,- 0.239546 },{ -0.23383, 0.440813, 0.866607 },{ 0.0678448 ,0.896541, - 0.437734 } };
vector<double> worldT3 = { -0.165381 ,-0.0071701 ,-1.82321 };
vector<vector<double>> worldR3 = { { -0.284939 ,0.47339, 0.833494 },{ 0.958402, 0.125654, 0.256273 },{ 0.0165852, 0.871845 ,- 0.489501 } };
vector<double> worldT4 = { 0.367129 ,0.160163 ,-1.74605 };
vector<vector<double>> worldR4 = { { 0.198585 ,- 0.376725 ,- 0.904788 },{ -0.979594, - 0.105479, - 0.171086 },{ -0.0309839, 0.9203 ,- 0.389984 } };
vector<vector<double>> worldT_all = { worldT1,worldT2,worldT3,worldT4 };
vector<vector<vector<double>>> worldR_all = { worldR1 ,worldR2,worldR3 ,worldR4 };


//vector<vector<double>> worldT_all = { worldT1,worldT2 };
//vector<vector<vector<double>>> worldR_all = { worldR1 ,worldR2};


pcl::PointXYZRGB RotatePoint(pcl::PointXYZRGB &point, std::vector<std::vector<double>> &R)
{
	pcl::PointXYZRGB res;

	res.x = point.x * R[0][0] + point.y * R[0][1] + point.z * R[0][2];
	res.y = point.x * R[1][0] + point.y * R[1][1] + point.z * R[1][2];
	res.z = point.x * R[2][0] + point.y * R[2][1] + point.z * R[2][2];

	return res;
}

int main(int argc, char** argv)

{
	int countimg =4;
	//读取./data/rgb.png和./data/depth.png，并转化为点云
	//图像矩阵
	cv::Mat rgb, depth;
	//使用cv::imread()来读取图像
	//rgb图像是8UC3的彩色图像
	//rgb = cv::imread("C:/vsprojects/marchingcubes/open3d3/Open3D/examples/Python/Advanced/realdata/four512424/rgb/0.png");
	//depth是16UC1的单通道图像，注意flags设置为-1，表示读取原始数据不做修改
	//depth = cv::imread("C:/vsprojects/marchingcubes/open3d3/Open3D/examples/Python/Advanced/realdata/four512424/depth/0.png", -1);


	//点云变量
	//使用智能指针，创建一个空点云。这种指针用完会自动释放
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//遍历深度图
	for (int ci = 0; ci < countimg; ++ci)
	{
		std::stringstream strrgb;
		strrgb << "C:/vsprojects/marchingcubes/open3d3/Open3D/examples/Python/Advanced/realdata/four512424/rgb/" << ci << ".png";
		rgb = cv::imread(strrgb.str());
		std::stringstream strdepth;
		strdepth << "C:/vsprojects/marchingcubes/open3d3/Open3D/examples/Python/Advanced/realdata/four512424/depth/" << ci << ".png";
		depth = cv::imread(strdepth.str(),-1);
		
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
				p.y = -(m - camera_cy)*p.z / camera_fy;

				//用外参将点从相机坐标系转化到世界坐标系中
			/*	p.x += worldT1[0];
				p.y += worldT1[1];
				p.z += worldT1[2];
				p = RotatePoint(p, worldR1);*/
				
				
				p.x += worldT_all[ci][0];
				p.y += worldT_all[ci][1] ;
				p.z += worldT_all[ci][2] ;
				p = RotatePoint(p, worldR_all[ci]);
				
				


				//从rgb图像中获取它的颜色
				//rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
				p.b = rgb.ptr<uchar>(m)[n * 3];
				p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
				p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

				//把p加入到点云中
				cloud->points.push_back(p);
			}
	}
	

	//设置并保存点云

	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout << "point cloud size=" << cloud->points.size() << endl;
	cloud->is_dense = false;

	//pcl::io::savePCDFile("C:/vsprojects/cvtest/cvtest/pointcloudyqy190509input.pcd", *cloud);
	pcl::PLYWriter writer;
	writer.write("C:/vsprojects/marchingcubes/open3d3/Open3D/examples/Python/Advanced/realdata/four512424/res/with_extr_para_4img_negY_camera_factor1000.ply", *cloud);

	//清楚数据并保存
	cloud->points.clear();
	cout << "Point cloud saved." << endl;

	return 0;

}
#endif
