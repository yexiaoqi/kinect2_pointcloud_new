#include "stdafx.h"
#if 0
//pcd���Ե����ϲ���
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

int main(int argc, char** argv)
{
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Load bun0.pcd -- should be available with the PCL archive in test 
	pcl::io::loadPCDFile("C:/vsprojects/test/test/007.pcd", *cloud);

	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);

	// Set parameters
	mls.setInputCloud(cloud);
	mls.setPolynomialOrder(2);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);

	// Reconstruct
	mls.process(mls_points);

	// Save output
	pcl::io::savePCDFile("C:/vsprojects/test/test/007upsample.pcd", mls_points);
}
#endif


#if 0
//�����ϲ�������

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/mls.h>


int main(int argc, char** argv)
{
	// �½����ƴ洢����
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	// ��ȡ�ļ�

	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("C:/vsprojects/test/test/test10000.ply", *cloud) != 0)
	{
		return -1;
	}
	// �˲�����
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> filter;
	filter.setInputCloud(cloud);
	//������������
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree;
	filter.setSearchMethod(kdtree);
	//������������İ뾶Ϊ3cm
	filter.setSearchRadius(0.005);
	// Upsampling �����ķ����� DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY,SAMPLE_LOCAL_PLANE
	filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::SAMPLE_LOCAL_PLANE);
	// �����İ뾶��
	filter.setUpsamplingRadius(0.005);
	// ���������Ĵ�С
	filter.setUpsamplingStepSize(2);

	filter.process(*filteredCloud);
	pcl::PLYWriter writer;
	writer.write("C:/vsprojects/test/test/test10000up.ply", *filteredCloud);
	return 0;
}
#endif



#if 0
//�����ϲ������������У���û������
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
using namespace std;
typedef pcl::PointXYZ point;
typedef pcl::PointCloud<point> pointcloud;


int main(int argc, char **argv)
{
	pointcloud::Ptr cloud(new pointcloud);
	//pcl::io::loadPCDFile("007_s.pcd", *cloud);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("007_s.pcd", *cloud) != 0)
	{
		cout << -1 << endl;;
	}

	cout << "points size is:" << cloud->size() << endl;
	pcl::search::KdTree<point>::Ptr tree(new pcl::search::KdTree<point>);

	//�����洢��mls����
	//    pcl::PointCloud<pcl::PointNormal> mls_points;
	pcl::PointCloud<point> mls_points;
	//pcl::PointCloud<point>::Ptr mls_points;

	//����mls����
	//  pcl::MovingLeastSquares<point,pcl::PointNormal> mls;

	pcl::MovingLeastSquares<point, point> mls;
	mls.setComputeNormals(true);
	mls.setInputCloud(cloud);
	//mls.setPolynomialFit(true); //����Ϊtrue����ƽ�������в��ö���ʽ�������߾���
	//mls.setPolynomialOrder(2); //MLS��ϵĽ�����Ĭ����2
	mls.setSearchMethod(tree);
	mls.setSearchRadius(1.1);


	// Upsampling �����ķ����� DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY,SAMPLE_LOCAL_PLANE
	mls.setUpsamplingMethod(pcl::MovingLeastSquares<point, point>::SAMPLE_LOCAL_PLANE);
	// �����İ뾶��
	mls.setUpsamplingRadius(4);
	// ���������Ĵ�С
	mls.setUpsamplingStepSize(4);



	std::cout << "before process" << endl;

	mls.process(mls_points);


	cout << "mls poits size is: " << mls_points.size() << endl;

	// Save output
	//pcl::io::savePCDFile("007up.pcd", mls_points);
	pcl::io::savePCDFile("007up_s.pcd", mls_points);
	return 0;
}

#endif





#if 0
//�����ϲ������������У������������
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
using namespace std;
typedef pcl::PointXYZ point;
typedef pcl::PointCloud<point> pointcloud;


int main(int argc, char **argv)
{

	// �½����ƴ洢����
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);


	//pointcloud::Ptr cloud(new pointcloud);
	//pcl::io::loadPCDFile("007_s.pcd", *cloud);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("007_s.pcd", *cloud) != 0)
	{
		cout << -1 << endl;;
	}

	cout << "points size is:" << cloud->size() << endl;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	//�����洢��mls����
	//    pcl::PointCloud<pcl::PointNormal> mls_points;
	pcl::PointCloud<pcl::PointXYZRGB> mls_points;
	//pcl::PointCloud<point>::Ptr mls_points;

	//����mls����
	//  pcl::MovingLeastSquares<point,pcl::PointNormal> mls;

	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
	mls.setComputeNormals(true);
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true); //����Ϊtrue����ƽ�������в��ö���ʽ�������߾���
	mls.setPolynomialOrder(2); //MLS��ϵĽ�����Ĭ����2
	mls.setSearchMethod(tree);
	mls.setSearchRadius(2);  //���ֵԽ������ĵ�Խ��
	std::cout << "before process" << endl;



	//mls.setSearchRadius(0.005);
	// Upsampling �����ķ����� DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY,SAMPLE_LOCAL_PLANE
	mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::SAMPLE_LOCAL_PLANE);
	// �����İ뾶��
	mls.setUpsamplingRadius(3);
	// ���������Ĵ�С
	mls.setUpsamplingStepSize(2);



	mls.process(mls_points);

	cout << "mls points size is: " << mls_points.size() << endl;

	// Save output
	//pcl::io::savePCDFile("007up.pcd", mls_points);
	pcl::io::savePCDFile("007up_s.pcd", mls_points);
	return 0;
}
#endif

