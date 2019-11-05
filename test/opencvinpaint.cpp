// compiletest.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#include <windows.h> 
#if 0
//计算处理时间
//opencv填补深度图空洞，采用opencv4.1.0后可以处理16位图像,这里是先把图像大小降为原来的1/5，inpaint后再变为原来大小，速度是0,3s/帧
#include <iostream>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\opencv.hpp>
using namespace std;
using namespace cv;

int main(int argc, char **argv) {
	SYSTEMTIME sTime1;
	SYSTEMTIME sTime2;

	int height = 1080;
	int width = 1920;
	Mat depthMat = cv::imread("C:/vsprojects/marchingcubes/open3d3/Open3D/examples/Python/Advanced/realdata/four/depth/1.png", -1);
	//Mat depthMat(height, width, CV_16UC1, depth); // from kinect
	Mat depthf(height, width, CV_8UC1);
	GetLocalTime(&sTime1);
	cout << sTime1.wHour << ": " << sTime1.wMinute << " :" << sTime1.wSecond << ". " << sTime1.wMilliseconds<<endl;

	depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);//oid Mat::convertTo( Mat& m, int rtype, double alpha=1, double beta=0 )alpha 缩放因子。默认值是1。即把原矩阵中的每一个元素都乘以alpha。beta 增量。默认值是0。即把原矩阵中的每一个元素都乘以alpha，再加上beta。
	//imshow("original-depth", depthf);
	//imwrite("C:/vsprojects/test/test/result3/depthinpaint/8bit2.png", depthf);
	const unsigned char noDepth = 0; // change to 255, if values no depth uses max value
	Mat temp, temp2;

	// 1 step - downsize for performance, use a smaller version of depth image这样比直接用原始大小图像处理快
	Mat small_depthf, small_depthf16;
	resize(depthf, small_depthf, Size(), 0.1, 0.1);
	resize(depthMat, small_depthf16, Size(), 0.1, 0.1);

	// 2 step - inpaint only the masked "unknown" pixels
	cv::inpaint(small_depthf16, (small_depthf == noDepth), temp, 5.0, INPAINT_TELEA);//第二个参数inpaintMask，图像的掩码，单通道图像，大小跟原图像一致，inpaintMask图像上除了需要修复的部分之外其他部分的像素值全部为0；(表明了需要修复的区域)

	// 3 step - upscale to original size and replace inpainted regions in original depth image
	resize(temp, temp2, depthMat.size());
	temp2.copyTo(depthMat, (depthMat == noDepth));  // add to the original signal
	GetLocalTime(&sTime2);
	cout << sTime2.wHour << ": " << sTime2.wMinute << " :" << sTime2.wSecond << ". " << sTime2.wMilliseconds<<endl;

	ULARGE_INTEGER fTime1;/*FILETIME*/
	ULARGE_INTEGER fTime2;/*FILETIME*/
	SystemTimeToFileTime(&sTime1, (FILETIME*)&fTime1);

	SystemTimeToFileTime(&sTime2, (FILETIME*)&fTime2);
	unsigned __int64 dft;
	dft = (fTime2.QuadPart - fTime1.QuadPart) / 10000;
	cout << dft;
	imshow("depth-inpaint", depthMat); // show results
	imwrite("C:/pycharmprojects/four/depth/inpaint1.png", depthMat);
	waitKey();
	return 0;
}
#endif


#if 0
//opencv填补深度图空洞，采用opencv4.1.0后可以处理16位图像，不缩放直接处理原来图像，约23s/帧
#include <iostream>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\opencv.hpp>
using namespace std;
using namespace cv;

int main(int argc, char **argv) {
	SYSTEMTIME sTime1;
	SYSTEMTIME sTime2;

	int height = 1080;
	int width = 1920;
	Mat depthMat = cv::imread("C:/vsprojects/test/test/result190624/result/4.png", -1);
	GetLocalTime(&sTime1);
	cout << sTime1.wHour << ": " << sTime1.wMinute << " :" << sTime1.wSecond << ". " << sTime1.wMilliseconds << endl;
	//Mat depthMat(height, width, CV_16UC1, depth); // from kinect
	Mat depthf(height, width, CV_8UC1);

	depthMat.convertTo(depthf, CV_8UC1);//缩放因子好像没啥影响
	/*depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);*///oid Mat::convertTo( Mat& m, int rtype, double alpha=1, double beta=0 )alpha 缩放因子。默认值是1。即把原矩阵中的每一个元素都乘以alpha。beta 增量。默认值是0。即把原矩阵中的每一个元素都乘以alpha，再加上beta。
														//imshow("original-depth", depthf);
														//imwrite("C:/vsprojects/test/test/result3/depthinpaint/8bit2.png", depthf);
	const unsigned char noDepth = 0; // change to 255, if values no depth uses max value
	Mat temp, temp2;

	// 1 step - downsize for performance, use a smaller version of depth image
	/*Mat small_depthf, small_depthf16;
	resize(depthf, small_depthf, Size(), 0.2, 0.2);
	resize(depthMat, small_depthf16, Size(), 0.2, 0.2);*/

	// 2 step - inpaint only the masked "unknown" pixels  inpaintRadius取5比较合适，太大了会使得上方头发模糊，人和墙中间有太多不应该有的连接线
	cv::inpaint(depthMat, (depthf == noDepth), temp, 5.0, INPAINT_TELEA);//第二个参数inpaintMask，图像的掩码，单通道图像，大小跟原图像一致，inpaintMask图像上除了需要修复的部分之外其他部分的像素值全部为0；(表明了需要修复的区域)

																					 // 3 step - upscale to original size and replace inpainted regions in original depth image
	resize(temp, temp2, depthMat.size());
	temp2.copyTo(depthMat, (depthMat == noDepth));  // add to the original signal
	GetLocalTime(&sTime2);
	cout << sTime2.wHour << ": " << sTime2.wMinute << " :" << sTime2.wSecond << ". " << sTime2.wMilliseconds << endl;

	ULARGE_INTEGER fTime1;/*FILETIME*/
	ULARGE_INTEGER fTime2;/*FILETIME*/
	SystemTimeToFileTime(&sTime1, (FILETIME*)&fTime1);

	SystemTimeToFileTime(&sTime2, (FILETIME*)&fTime2);
	unsigned __int64 dft;
	dft = (fTime2.QuadPart - fTime1.QuadPart) / 10000;
	cout << dft;
	imshow("depth-inpaint", depthMat); // show results
	imwrite("C:/vsprojects/test/test/result190624/result/4inpaint4.png", depthMat);
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

