#include "stdafx.h"
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
	//depth = cv::imread("C:/vsprojects/test/test/inputimg190519/depth/1.png", -1);
	depth = cv::imread("C:/vsprojects/kinect_depth_inpainting_and_filtering/kinect_depth_inpainting_and_filtering/inpaint.png", -1);
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
	writer.write("C:/vsprojects/test/test/16bitonlyinpaint190530.ply", *cloud);

	//清楚数据并保存

	cloud->points.clear();

	cout << "Point cloud saved." << endl;

	return 0;

}
#endif


#if 0
//8位深度图转点云
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
	//depth = cv::imread("C:/vsprojects/test/test/inputimg190519/depth/1.png", -1);
	/*depth = cv::imread("C:/vsprojects/test/test/result3/depthinpaint/2.png",-1);*/
	depth = cv::imread("C:/vsprojects/kinect_depth_inpainting_and_filtering/kinect_depth_inpainting_and_filtering/filtered median.png", -1);

	//rgb = cv::imread("C:/vsprojects/cvtest/cvtest/20170907/group1/color_map/frame_000001.png");

	////depth是16UC1的单通道图像，注意flags设置为-1，表示读取原始数据不做修改

	//depth = cv::imread("C:/vsprojects/cvtest/cvtest/20170907/group1/depth_map/frame_000001.png", -1);

	//点云变量

	//使用智能指针，创建一个空点云。这种指针用完会自动释放

	//PointCloud::Ptr cloud(new PointCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//遍历深度图

	for (int m = 0; m < depth.rows; m++)

		for (int n = 0; n < depth.cols; n++)

		{

			//获取深度图中(m,n)处的值

			//ushort d = depth.ptr<ushort>(m)[n];
			uchar d = depth.ptr<uchar>(m)[n];//8位要改为uchar！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！

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
			//p.z = double(d)/ camera_factor;

			//p.x = n;

			//p.y = m ;

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
	writer.write("C:/vsprojects/test/test/onlymidfilter190529.ply", *cloud);

	//清楚数据并保存

	cloud->points.clear();

	cout << "Point cloud saved." << endl;

	return 0;
}
#endif



#if 0
//深度图和rgb图转化为点云,存为pcd格式
#include<iostream>

#include<string>

using namespace std;

//opencv库
#include <opencv2/opencv.hpp>
//#include<opencv2/core/core.cpp>

//#include<opencv2/highgui/highgui.hpp>

//PCL库

#include<pcl/io/pcd_io.h>

#include<pcl/point_types.h>






//定义点云类型

typedef pcl::PointXYZRGBA PointT;

typedef pcl::PointCloud<PointT> PointCloud;

//相机内参
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

//主函数

int main(int argc, char** argv)

{

	//读取./data/rgb.png和./data/depth.png，并转化为点云

	//图像矩阵

	cv::Mat rgb, depth;

	//使用cv::imread()来读取图像

	//rgb图像是8UC3的彩色图像

	rgb = cv::imread("C:/vsprojects/test/test/result/rgb2/1.png");

	//depth是16UC1的单通道图像，注意flags设置为-1，表示读取原始数据不做修改

	/*depth = cv::imread("C:/vsprojects/cvtest/cvtest/190509yqy/dt_1.png", -1);*/
	depth = cv::imread("C:/vsprojects/test/test/result/depth/1.png", -1);
	//rgb = cv::imread("C:/vsprojects/cvtest/cvtest/20170907/group1/color_map/frame_000001.png");

	////depth是16UC1的单通道图像，注意flags设置为-1，表示读取原始数据不做修改

	//depth = cv::imread("C:/vsprojects/cvtest/cvtest/20170907/group1/depth_map/frame_000001.png", -1);

	//点云变量

	//使用智能指针，创建一个空点云。这种指针用完会自动释放

	PointCloud::Ptr cloud(new PointCloud);

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

			PointT p;

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

	pcl::io::savePCDFile("C:/vsprojects/test/test/pointcloud/forunsample190603.pcd", *cloud);

	//清楚数据并保存

	cloud->points.clear();

	cout << "Point cloud saved." << endl;

	return 0;

}
#endif





//针对和彩色图对齐的1920*1080深度图，将其转化为点云，不使用内参直接映射
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

//主函数

int main(int argc, char** argv)

{

	//读取./data/rgb.png和./data/depth.png，并转化为点云

	//图像矩阵

	cv::Mat rgb, depth;

	//使用cv::imread()来读取图像

	//rgb图像是8UC3的彩色图像

	rgb = cv::imread("C:/vsprojects/test/test/result190604/rgb19201080/19.png");

	//depth是16UC1的单通道图像，注意flags设置为-1，表示读取原始数据不做修改

	/*depth = cv::imread("C:/vsprojects/cvtest/cvtest/190509yqy/dt_1.png", -1);*/
	//depth = cv::imread("C:/vsprojects/test/test/imgresult/2.png", -1);
	depth = cv::imread("C:/vsprojects/Robust-Color-Guided-Depth-Map-Restoration-master/Robust-Color-Guided-Depth-Map-Restoration-master/yqy/2inpaintopencv.png", -1);
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
			p.z = double(d) ;

			p.x = n ;

			p.y = m;

			/*p.z = double(d) / camera_factor;

			p.x = (n - camera_cx)*p.z / camera_fx;

			p.y = (m - camera_cy)*p.z / camera_fy;*/

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
	writer.write("C:/vsprojects/test/test/result190604/190606172inpaintopencv.ply", *cloud);

	//清楚数据并保存

	cloud->points.clear();

	cout << "Point cloud saved." << endl;

	return 0;

}
#endif