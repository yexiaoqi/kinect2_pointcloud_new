#include "stdafx.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <pcl/visualization/cloud_viewer.h>  

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
#if 0
int main()
{
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	////pcl::io::loadPCDFile("model1.pcd",*cloud);  

	////ply�ļ���ʾ  
	//pcl::PolygonMesh mesh;
	//vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

	//pcl::io::loadPolygonFilePLY("raw.ply", mesh);
	//// ply���vtk  
	////pcl::io::saveVTKFile("temp.vtk", mesh);  
	//pcl::io::mesh2vtk(mesh, polydata);

	//pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);

	////���ִ�����ʽ pcd���pcd  
	//pcl::PCDWriter pcdwriter;
	////pcdwriter.write<pcl::PointXYZRGBA>("save_ply2vtk2pcd.pcd", *cloud);  
	//pcl::io::savePCDFileASCII("raw1.pcd", *cloud);

	//*VCGLIB���� ��ply
	/*pcl::PCLPointCloud2 clod;
	pcl::io::loadPLYFile("raw.ply", clod);
	pcl::io::savePCDFile("raw11.pcd", clod);*/






	//pcl::PLYReader reader;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::io::loadPLYFile<pcl::PointXYZRGB>("./xiaowen.ply", *cloud);
	////reader.read<pcl::PointXYZRGB>("./xiaowen.ply", *cloud);
	//pcl::io::savePCDFile("./xiaowen.pcd", *cloud);


	pcl::PCLPointCloud2 clod;
	pcl::PLYReader reader;
	//reader.read("./xiaowen.ply", clod);
	reader.read("./level7/1234kinectcenter191017_225_2_ten_deletefloor8.ply", clod);

	pcl::PCDWriter writer;
	//writer.writeASCII("./xiaowen.pcd", clod);
	writer.writeASCII("./level7/1234kinectcenter191017_225_2_ten_deletefloor8.pcd", clod);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	//����ply�ļ��и�ʽΪRGBA��ʽ�����Ͼ��дΪ
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	//pcl::io::loadPCDFile("./xiaowen.pcd", *cloud);
	pcl::io::loadPCDFile("./level7/1234kinectcenter191017_225_2_ten_deletefloor8.pcd", *cloud);


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewe(new pcl::visualization::PCLVisualizer("ss"));
	//viewe->initCameraParameters();
	viewe->setBackgroundColor(0, 0, 0);
	//viewe->addCoordinateSystem(1.0f);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(cloud);
	//����ply�ļ��и�ʽΪRGBA��ʽ�����Ͼ��дΪ
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> color(cloud);
	viewe->addPointCloud<pcl::PointXYZRGB>(cloud, color, "cloud");
	//����ply�ļ��и�ʽΪRGBA��ʽ�����Ͼ��дΪ
	//viewe->addPointCloud<pcl::PointXYZRGBA>(cloud, color, "cloud");

	//viewe->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 2, "cloud");
	while (!viewe->wasStopped()) {
		viewe->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}
#endif