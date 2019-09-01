#include "stdafx.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#if 0
//去除离散噪声点，效果不错
int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	// 定义读取对象
	pcl::PCDReader reader;
	// 读取点云文件
	reader.read<pcl::PointXYZRGB>("xiaowenstatisticalOutlierRemoval_inlier190820.pcd", *cloud);

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
	//个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;   //创建滤波器对象
	sor.setInputCloud(cloud);                           //设置待滤波的点云
	sor.setMeanK(100);                               //设置在进行统计时考虑查询点临近点数
	sor.setStddevMulThresh(0.01);                      //设置判断是否为离群点的阀值
	sor.filter(*cloud_filtered);                    //存储

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZRGB>("./xiaowenstatisticalOutlierRemoval_inlier190820again.pcd", *cloud_filtered, false);//去除离群点后的点


	//sor.setNegative(true);
	//sor.filter(*cloud_filtered);
	//writer.write<pcl::PointXYZRGB>("./xiaowenstatisticalOutlierRemoval_outlier.pcd", *cloud_filtered, false);//仅有离群点

	return (0);
}
#endif