#include "stdafx.h"

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
	string input_filename = "C:/vsprojects/test/test/007up_s.pcd";
	string output_filename = "C:/vsprojects/test/test/007up_s.ply";
	PCDtoPLYconvertor(input_filename, output_filename);
	return 0;
}
#endif
