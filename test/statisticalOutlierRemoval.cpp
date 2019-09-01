#include "stdafx.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#if 0
//ȥ����ɢ�����㣬Ч������
int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	// �����ȡ����
	pcl::PCDReader reader;
	// ��ȡ�����ļ�
	reader.read<pcl::PointXYZRGB>("xiaowenstatisticalOutlierRemoval_inlier190820.pcd", *cloud);

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// �����˲�������ÿ����������ٽ���ĸ�������Ϊ50 ��������׼��ı�������Ϊ1  ����ζ�����һ
	//����ľ��볬����ƽ������һ����׼�����ϣ���õ㱻���Ϊ��Ⱥ�㣬�������Ƴ����洢����
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;   //�����˲�������
	sor.setInputCloud(cloud);                           //���ô��˲��ĵ���
	sor.setMeanK(100);                               //�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ�����
	sor.setStddevMulThresh(0.01);                      //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ
	sor.filter(*cloud_filtered);                    //�洢

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZRGB>("./xiaowenstatisticalOutlierRemoval_inlier190820again.pcd", *cloud_filtered, false);//ȥ����Ⱥ���ĵ�


	//sor.setNegative(true);
	//sor.filter(*cloud_filtered);
	//writer.write<pcl::PointXYZRGB>("./xiaowenstatisticalOutlierRemoval_outlier.pcd", *cloud_filtered, false);//������Ⱥ��

	return (0);
}
#endif