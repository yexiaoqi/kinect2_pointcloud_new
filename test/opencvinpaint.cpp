#include "stdafx.h"

#if 0
//opencv填补深度图空洞，采用opencv4.1.0后可以处理16位图像
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
	Mat depthMat = cv::imread("C:/vsprojects/Robust-Color-Guided-Depth-Map-Restoration-master/Robust-Color-Guided-Depth-Map-Restoration-master/yqy/2.png", -1);
	//Mat depthMat(height, width, CV_16UC1, depth); // from kinect
	Mat depthf(height, width, CV_8UC1);

	depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);
	//imshow("original-depth", depthf);
	//imwrite("C:/vsprojects/test/test/result3/depthinpaint/8bit2.png", depthf);
	const unsigned char noDepth = 0; // change to 255, if values no depth uses max value
	Mat temp, temp2;

	// 1 step - downsize for performance, use a smaller version of depth image
	Mat small_depthf, small_depthf16;
	resize(depthf, small_depthf, Size(), 0.2, 0.2);
	resize(depthMat, small_depthf16, Size(), 0.2, 0.2);

	// 2 step - inpaint only the masked "unknown" pixels
	cv::inpaint(small_depthf16, (small_depthf == noDepth), temp, 5.0, INPAINT_TELEA);

	// 3 step - upscale to original size and replace inpainted regions in original depth image
	resize(temp, temp2, depthMat.size());
	temp2.copyTo(depthMat, (depthMat == noDepth));  // add to the original signal

	imshow("depth-inpaint", depthMat); // show results
	imwrite("C:/vsprojects/Robust-Color-Guided-Depth-Map-Restoration-master/Robust-Color-Guided-Depth-Map-Restoration-master/yqy/2inpaintopencv.png", depthMat);
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

