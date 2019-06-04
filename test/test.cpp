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
//opencv填补深度图空洞，但只能处理8位图像
#include <iostream>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\opencv.hpp>
using namespace std;
using namespace cv;

int main(int argc, char **argv) {
	int height = 424;
	int width = 512;
	Mat depthMat = cv::imread("C:/vsprojects/test/test/result3/depth/2.png", -1);
	//Mat depthMat(height, width, CV_16UC1, depth); // from kinect
	Mat depthf(height, width, CV_8UC1);
	
	depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);
	imshow("original-depth", depthf);
	imwrite("C:/vsprojects/test/test/result3/depthinpaint/8bit2.png", depthf);
	const unsigned char noDepth = 0; // change to 255, if values no depth uses max value
	Mat temp, temp2;

	// 1 step - downsize for performance, use a smaller version of depth image
	Mat small_depthf;
	resize(depthf, small_depthf, Size(), 0.2, 0.2);

	// 2 step - inpaint only the masked "unknown" pixels
	cv::inpaint(small_depthf, (small_depthf == noDepth), temp, 5.0, INPAINT_TELEA);

	// 3 step - upscale to original size and replace inpainted regions in original depth image
	resize(temp, temp2, depthf.size());
	temp2.copyTo(depthf, (depthf == noDepth));  // add to the original signal

	imshow("depth-inpaint", depthf); // show results
	imwrite("C:/vsprojects/test/test/result3/depthinpaint/2.png", depthf);
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

