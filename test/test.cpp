#include "stdafx.h"

#if 0
//对一幅图像进行二值化

#include <iostream>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
	Mat imag, result;
	imag = imread("C:/vsprojects/test/test/inputimg190519/depth/1.png", 0);	//将读入的彩色图像直接以灰度图像读入
	namedWindow("原图", 1);
	imshow("原图", imag);
	result = imag.clone();
	//进行二值化处理，选择30，200.0为阈值
	threshold(imag, result, 0, 200.0, THRESH_BINARY);
	namedWindow("二值化图像");
	imshow("二值化图像", result);
	waitKey();
	return 0;
}
#endif






#if 0
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

#include <iostream>

int main(int argc, char** argv)
{
	// read a image and a pcd
	cv::Mat image_origin = cv::imread("C:/vsprojects/test/test/result/rgb/1.png");
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_origin(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_withoutNAN(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::io::loadPCDFile<pcl::PointXYZI>("C:/vsprojects/test/test/pointcloud/forunsample190603.pcd", *cloud_origin);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud_origin, *cloud_withoutNAN, indices);

	std::vector<cv::Point3f> pts_3d;
	for (size_t i = 0; i < cloud_withoutNAN->size(); ++i)
	{
		pcl::PointXYZI point_3d = cloud_withoutNAN->points[i];
		//if (point_3d.x > 2 && point_3d.x < 3 && point_3d.y > -10 && point_3d.y < 10)
		{
			pts_3d.emplace_back(cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
		}
	}

	// using iterator

	// read calibration parameter
	double fx = 1.0757955405501191e+03, fy = 1.0762345733674481e+03;
	double cx = 9.6249394948422218e+02, cy = 6.1957628038839391e+02;
	double k1 = -1.1995613777994101e-01, k2 = 8.6245969435724004e-02, k3 = -2.6778267188218002e-02;
	double p1 = 1.0621717082800000e-03, p2 = 5.4033385896265832e-04;
	cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
	cv::Mat distortion_coeff = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
	cv::Mat r_vec = (cv::Mat_<double>(3, 1) << 1.29949179254383, -1.113823535227475, 1.108412921650477);
	cv::Mat t_vec = (cv::Mat_<double>(3, 1) << -0.370740907093656, -0.2397403632299851, -0.0407927826288379);

	// project 3d-points into image view
	std::vector<cv::Point2f> pts_2d;
	cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff, pts_2d);
	cv::Mat image_project = image_origin.clone();
	int image_rows = image_origin.rows;
	int image_cols = image_origin.cols;

	for (size_t i = 0; i < pts_2d.size(); ++i)
	{
		cv::Point2f point_2d = pts_2d[i];
		if (point_2d.x < 0 || point_2d.x > image_cols || point_2d.y < 0 || point_2d.y > image_rows)
		{
			continue;
		}
		else
		{
			image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[0] = 0;
			image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[1] = 0;
			image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[2] = 255;
		}

		if (point_2d.x > 0 && point_2d.x < image_cols && point_2d.y > 0 && point_2d.y < image_rows)
		{
			image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[0] = 0;
			image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[1] = 0;
			image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[2] = 255;
		}
		else
		{
			continue;
		}
	}

	cv::imshow("origin image", image_origin);
	cv::imshow("project image", image_project);
	cv::imwrite("C:/vsprojects/test/test/imgresult/image_origin.png", image_origin);
	cv::imwrite("C:/vsprojects/test/test/imgresult/image_project.png", image_project);
	cv::waitKey(10000);

	return 0;
}
#endif

#if 0
//将图像转为CV_8UC3格式
#include <iostream>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
	Mat imag(1080, 1920, CV_8UC3);
	imag = imread("C:/vsprojects/test/test/result190604/rgb19201080/19.png");	
	
	/*imshow("test",imag);
	waitKey(0);*/
	imwrite("24bit.png", imag);
	return 0;
}
#endif




#if 0
//联合双边滤波 只能用于8u或32f
#include <opencv2/opencv.hpp>
#include <ximgproc.hpp>

int main()
{
	cv::Mat src = cv::imread("C:/vsprojects/Robust-Color-Guided-Depth-Map-Restoration-master/Robust-Color-Guided-Depth-Map-Restoration-master/yqy/2inpaintopencv.png", 0); // 原始带噪声的深度图
	cv::Mat joint = cv::imread("C:/vsprojects/Robust-Color-Guided-Depth-Map-Restoration-master/Robust-Color-Guided-Depth-Map-Restoration-master/yqy/2inpaintopencvresize.png", 0);

	cv::Mat dst;
	//int64 begin = cvGetTickCount();
	cv::ximgproc::jointBilateralFilter(joint, src, dst, -1, 3, 9);
	//int64 end = cvGetTickCount();

	//float time = (end - begin) / (cvGetTickFrequency() * 1000.);
	//printf("time = %fms\n", time);

	imshow("src", src);
	imshow("joint", joint);
	imshow("jointBilateralFilter", dst);
	imwrite("C:/vsprojects/Robust-Color-Guided-Depth-Map-Restoration-master/Robust-Color-Guided-Depth-Map-Restoration-master/yqy/src.png", src);
	imwrite("C:/vsprojects/Robust-Color-Guided-Depth-Map-Restoration-master/Robust-Color-Guided-Depth-Map-Restoration-master/yqy/joint.png", joint);
	imwrite("C:/vsprojects/Robust-Color-Guided-Depth-Map-Restoration-master/Robust-Color-Guided-Depth-Map-Restoration-master/yqy/2jointBilateralFilter.png", dst);
	cv::waitKey(0);


	return 0;
}
#endif