// compiletest.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"

#include <windows.h> 
#if 0
//���㴦��ʱ��
//opencv����ͼ�ն�������opencv4.1.0����Դ���16λͼ��,�������Ȱ�ͼ���С��Ϊԭ����1/5��inpaint���ٱ�Ϊԭ����С���ٶ���0,3s/֡
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

	depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);//oid Mat::convertTo( Mat& m, int rtype, double alpha=1, double beta=0 )alpha �������ӡ�Ĭ��ֵ��1������ԭ�����е�ÿһ��Ԫ�ض�����alpha��beta ������Ĭ��ֵ��0������ԭ�����е�ÿһ��Ԫ�ض�����alpha���ټ���beta��
	//imshow("original-depth", depthf);
	//imwrite("C:/vsprojects/test/test/result3/depthinpaint/8bit2.png", depthf);
	const unsigned char noDepth = 0; // change to 255, if values no depth uses max value
	Mat temp, temp2;

	// 1 step - downsize for performance, use a smaller version of depth image������ֱ����ԭʼ��Сͼ�����
	Mat small_depthf, small_depthf16;
	resize(depthf, small_depthf, Size(), 0.1, 0.1);
	resize(depthMat, small_depthf16, Size(), 0.1, 0.1);

	// 2 step - inpaint only the masked "unknown" pixels
	cv::inpaint(small_depthf16, (small_depthf == noDepth), temp, 5.0, INPAINT_TELEA);//�ڶ�������inpaintMask��ͼ������룬��ͨ��ͼ�񣬴�С��ԭͼ��һ�£�inpaintMaskͼ���ϳ�����Ҫ�޸��Ĳ���֮���������ֵ�����ֵȫ��Ϊ0��(��������Ҫ�޸�������)

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
//opencv����ͼ�ն�������opencv4.1.0����Դ���16λͼ�񣬲�����ֱ�Ӵ���ԭ��ͼ��Լ23s/֡
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

	depthMat.convertTo(depthf, CV_8UC1);//�������Ӻ���ûɶӰ��
	/*depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);*///oid Mat::convertTo( Mat& m, int rtype, double alpha=1, double beta=0 )alpha �������ӡ�Ĭ��ֵ��1������ԭ�����е�ÿһ��Ԫ�ض�����alpha��beta ������Ĭ��ֵ��0������ԭ�����е�ÿһ��Ԫ�ض�����alpha���ټ���beta��
														//imshow("original-depth", depthf);
														//imwrite("C:/vsprojects/test/test/result3/depthinpaint/8bit2.png", depthf);
	const unsigned char noDepth = 0; // change to 255, if values no depth uses max value
	Mat temp, temp2;

	// 1 step - downsize for performance, use a smaller version of depth image
	/*Mat small_depthf, small_depthf16;
	resize(depthf, small_depthf, Size(), 0.2, 0.2);
	resize(depthMat, small_depthf16, Size(), 0.2, 0.2);*/

	// 2 step - inpaint only the masked "unknown" pixels  inpaintRadiusȡ5�ȽϺ��ʣ�̫���˻�ʹ���Ϸ�ͷ��ģ�����˺�ǽ�м���̫�಻Ӧ���е�������
	cv::inpaint(depthMat, (depthf == noDepth), temp, 5.0, INPAINT_TELEA);//�ڶ�������inpaintMask��ͼ������룬��ͨ��ͼ�񣬴�С��ԭͼ��һ�£�inpaintMaskͼ���ϳ�����Ҫ�޸��Ĳ���֮���������ֵ�����ֵȫ��Ϊ0��(��������Ҫ�޸�������)

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
//opencv����ͼ�ն�����ֻ�ܴ���8λͼ��
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

