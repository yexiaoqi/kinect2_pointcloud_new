#include "stdafx.h"
/*
���ظ� ������������ռ��� ����һ���㣨���ĵ㣩��
�������ӵ�������
https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd
���ظ��˲���VoxelGrid�����������ڼ��ٵ�������֤����λ�ò��䡡PCLPointCloud2()
�²���
#include <pcl/filters/voxel_grid.h>
ע��˵�������Ϊ��pcl::PCLPointCloud2������  blob����������
#include <pcl/filters/voxel_grid.h>
// ת��Ϊģ����� pcl::PointCloud<pcl::PointXYZ>
pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
���ʹ�ø߷ֱ���������豸�Ե��ƽ��вɼ����������ƻ��Ϊ�ܼ���
����ĵ���������Ժ����ָ���������ѡ�
���ظ��˲������Դﵽ���²���ͬʱ���ƻ����Ʊ����νṹ�Ĺ��ܡ�
���Ƽ��νṹ �����Ǻ�۵ļ������Σ�Ҳ������΢�۵����з�ʽ��
����������Ƶĳߴ磬������ͬ�ľ��롣
����²�����ȻЧ�ʱ������˲����ߣ������ƻ�����΢�۽ṹ.
ʹ�����ػ����񷽷�ʵ���²����������ٵ������ ���ٵ������ݣ�
��ͬʱ������Ƶ���״�������������׼�������ؽ�����״ʶ����㷨�ٶ��зǳ�ʵ�ã�
PCL��ʵ�ֵ�VoxelGrid��ͨ������ĵ������ݴ���һ����ά����դ��
���ɺ�ÿ�������������������е��������������ʾ�����������㣬
���������������е㶼��һ�����ĵ����ձ�ʾ�������������ش����õ��Ĺ��˺�ĵ��ƣ�
���ַ��������������ģ�ע�����ĺ����ģ��ƽ��ķ������������Ƕ��ڲ������Ӧ����ı�ʾ��Ϊ׼ȷ��
*/
#include <iostream>
#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>//���ظ��˲���VoxelGrid
#include <pcl/io/pcd_io.h>//�����ļ�pcd ��д
#include <pcl/visualization/cloud_viewer.h>//���ƿ��ӻ�
//#include <pcl_conversions/pcl_conversions.h>//��������ת��


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/mls.h>
/*
CloudViewer�Ǽ���ʾ���ƵĿ��ӻ����ߣ�����ʹ�ñȽ��ٵĴ���鿴���ƣ�
��������ǲ������ڶ��߳�Ӧ�ó����еġ�
����Ĵ���Ĺ����ǹ�������ڿ��ӻ��߳������д�������ӣ�
PCLVisualizer��CloudViewer�ĺ�ˣ��������Լ����߳������У�
���Ҫʹ��PCLVisualizer�����ʹ�õ��ú������������Ա�����ӻ��Ĳ������⡣
������ʵ�ʵ��õ�ʱ��Ҫע�⣬�Է����ֺ�����ת����һ����鷳�����⡣
*/
using namespace std;
// ����
typedef pcl::PointCloud<pcl::PointXYZ>  Cloud;

#if 0
int main(int argc, char** argv)
{
	// ���塡���ƶ���ָ��
	pcl::PCLPointCloud2::Ptr cloud2_ptr(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud2_filtered_ptr(new pcl::PCLPointCloud2());
	Cloud::Ptr cloud_filtered_ptr(new Cloud);
	// ��ȡ�����ļ��������ƶ���
	pcl::PCDReader reader;
	reader.read("./xiaowen.pcd", *cloud2_ptr);
	if (cloud2_ptr == NULL) { cout << "pcd file read err" << endl; return -1; }
	cout << "PointCLoud before filtering �˲�ǰ����: " << cloud2_ptr->width * cloud2_ptr->height
		<< " data points ( " << pcl::getFieldsList(*cloud2_ptr) << "." << endl;

	// �����˲�������Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
	// pcl::ApproximateVoxelGrid<pcl::PointXYZ> avg;
	vg.setInputCloud(cloud2_ptr);//�����������
	vg.setLeafSize(0.01f, 0.01f, 0.01f);//�����ؿ��С����cm
	vg.filter(*cloud2_filtered_ptr);


	// ����˲���ĵ�����Ϣ
	cout << "PointCLoud before filtering �˲�������: " << cloud2_filtered_ptr->width * cloud2_filtered_ptr->height
		<< " data points ( " << pcl::getFieldsList(*cloud2_filtered_ptr) << "." << endl;
	// д���ڴ�
	pcl::PCDWriter writer;
	writer.write("xiaowen_downsampled.pcd", *cloud2_filtered_ptr,
		Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

	// ����ϵͳ���ӻ���������ʾ
	//system("pcl_viewer table_scene_lms400_inliers.pcd");

	// ת��Ϊģ����� pcl::PointCloud<pcl::PointXYZ>
	pcl::fromPCLPointCloud2(*cloud2_filtered_ptr, *cloud_filtered_ptr);

	// ������ӻ�
	pcl::visualization::CloudViewer viewer("pcd��viewer");// ��ʾ���ڵ�����
	viewer.showCloud(cloud_filtered_ptr);
	while (!viewer.wasStopped())
	{
		// Do nothing but wait.
	}

	return (0);
}
#endif


