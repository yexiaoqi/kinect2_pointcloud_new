#include "stdafx.h"


//pcd转ply文件成功
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include<pcl/PCLPointCloud2.h>
#include<iostream>
#include<string>

using namespace pcl;
using namespace pcl::io;
using namespace std;

#if 0
//不知道为啥之前应该是成功的，但现在转化出来每个点之间的距离相距特别远
int PCDtoPLYconvertor(string & input_filename, string& output_filename)
{
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud;
	//pcl::PointCloud<pcl::PointXYZRGB> &cloud;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PCLPointCloud2 cloud;
	if (loadPCDFile(input_filename, cloud) < 0)
	{
		cout << "Error: cannot load the PCD file!!!" << endl;
		return -1;
	}
	PLYWriter writer;
	writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true, true);
	//writer.write(output_filename, cloud, true, true);
	return 0;

}

int main()
{
	//string input_filename = "C:/vsprojects/cvtest/cvtest/pointcloud.pcd";
	//string output_filename = "C:/vsprojects/cvtest/cvtest/pointcloud.ply";
	string input_filename = "./xiaowen_downsampled.pcd";
	string output_filename = "./xiaowen_downsampled4.ply";
	PCDtoPLYconvertor(input_filename, output_filename);
	return 0;
}
#endif


//成功
int main()
{
	//string input_filename = "C:/vsprojects/cvtest/cvtest/pointcloud.pcd";
	//string output_filename = "C:/vsprojects/cvtest/cvtest/pointcloud.ply";
	string input_filename = "./xiaowenstatisticalOutlierRemoval1.pcd";
	string output_filename = "./xiaowenstatisticalOutlierRemoval5.ply";
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	if (loadPCDFile(input_filename,*cloud) < 0)
	{
		cout << "Error: cannot load the PCD file!!!" << endl;
		return -1;
	}
	PLYWriter writer;
	writer.write(output_filename, *cloud);
	
	return 0;
}