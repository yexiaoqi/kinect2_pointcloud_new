#include "stdafx.h"

#if 0
//生成点云成功,按s保存点云,这段代码得到的pcd可以转ply
#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGBA PointType;

int main(int argc, char* argv[])
{
	// PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
		new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);

	// Point Cloud
	pcl::PointCloud<PointType>::ConstPtr cloud;

	// Retrieved Point Cloud Callback Function
	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
		[&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
		boost::mutex::scoped_lock lock(mutex);

		/* Point Cloud Processing */

		cloud = ptr->makeShared();
	};
	// Kinect2Grabber
	//  boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();
	boost::shared_ptr<pcl::Grabber> grabber = boost::shared_ptr<pcl::Grabber>(new pcl::Kinect2Grabber);
	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);


	// Keyboard Callback Function
	boost::function<void(const pcl::visualization::KeyboardEvent&)> keyboard =
		[&cloud, &mutex](const pcl::visualization::KeyboardEvent& event) {
		if (event.getKeySym() == "s" && event.keyDown()) {
			boost::mutex::scoped_lock lock(mutex);

			// Generate Indices ( 000, 001, 002, ... )
			static uint32_t index = 0;
			std::ostringstream oss;
			oss << std::setfill('0') << std::setw(3) << index++;
			std::cout << oss.str() + ".ply" << std::endl;

			// Save Point Cloud to PCD File
			//pcl::io::savePCDFile(oss.str() + ".pcd", *cloud);
			pcl::PLYWriter writer;
			writer.write(oss.str() + ".ply", *cloud);
		}
	};




	// Start Grabber
	grabber->start();
	int count = 0;
	while (!viewer->wasStopped()) {
		// Update Viewer
		viewer->spinOnce();

		boost::mutex::scoped_try_lock lock(mutex);
		// Register Keyboard Callback Function
		viewer->registerKeyboardCallback(keyboard);
		//if (count == 0)
		//{
		//	pcl::PLYWriter writer;
		//	writer.write("yqyou34363.ply", *cloud);
		//	//ofstream OutFile("zyd0519.txt");

		//	////OutFile << "This is a Test12!";

		//	//for (int ind = 0; ind < cloud->size(); ind++)
		//	//{
		//	//	//OutFile << cloud->points.at(ind) << endl;
		//	//	OutFile << cloud->at(ind) << endl;
		//	//	//cout << ind << endl;
		//	//	//cout << cloud->points.at(ind) << endl;
		//	//}

		//	//OutFile.close();
		//	count += 1;
		//}

		if (lock.owns_lock() && cloud) {
			// Update Point Cloud
			if (!viewer->updatePointCloud(cloud, "cloud")) {
				viewer->addPointCloud(cloud, "cloud");
				//cout << cloud->size() << endl; //217088
				//cout << cloud->points.size() << endl; //217088

				//cout << cloud->points.at(0) << endl;

			}

		}
	}
	// Stop Grabber
	grabber->stop();

	// Disconnect Callback Function
	if (connection.connected()) {
		connection.disconnect();
	}

	return 0;
}
#endif




#if 0
//直接用kinect采点云存为ply
//生成点云成功,但只能是静止物体
#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <kinect.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include<pcl/PCLPointCloud2.h>

typedef pcl::PointXYZRGB PointType;

int main(int argc, char* argv[])
{
	// PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
		new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);

	// Point Cloud
	pcl::PointCloud<PointType>::ConstPtr cloud;

	// Retrieved Point Cloud Callback Function
	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
		[&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
		boost::mutex::scoped_lock lock(mutex);

		/* Point Cloud Processing */

		cloud = ptr->makeShared();
	};
	// Kinect2Grabber
	//  boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();
	boost::shared_ptr<pcl::Grabber> grabber = boost::shared_ptr<pcl::Grabber>(new pcl::Kinect2Grabber);
	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);

	// Start Grabber
	grabber->start();
	int count = 0;
	while (!viewer->wasStopped()) {
		// Update Viewer
		viewer->spinOnce();

		boost::mutex::scoped_try_lock lock(mutex);

		/*pcl::PLYWriter writer;
		writer.write("kinectcloud190519.ply", *cloud);*/
		//writer.writeBinary("kinectcloud190519.ply", cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true);
		pcl::io::savePCDFileASCII("kinectcloud190522.pcd", *cloud);
		//if (count == 0)
		//{
		//	pcl::PLYWriter writer;
		//	writer.write("kinectcloud190519.ply", *cloud);
		//	
		//	//pcl::io::savePCDFileASCII("kinectcloud190519.ply",*cloud);
		//	count += 1;
		//}

		if (lock.owns_lock() && cloud) {
			// Update Point Cloud
			if (!viewer->updatePointCloud(cloud, "cloud")) {
				viewer->addPointCloud(cloud, "cloud");
				//if (count == 0)
				//{
				//	
				//	pcl::io::savePCDFileASCII("kinectcloud1905192.ply", *cloud);
				//	//count += 1;
				//}

			}
		}
	}
	// Stop Grabber
	grabber->stop();

	// Disconnect Callback Function
	if (connection.connected()) {
		connection.disconnect();
	}

	return 0;
}
#endif

