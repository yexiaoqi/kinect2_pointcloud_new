#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Kinect.h>

using namespace std;
using namespace cv;

int main(void)
{
	// 1a.获取感应器
	IKinectSensor* pSensor = nullptr;
	GetDefaultKinectSensor(&pSensor);
	// 1b. 打开感应器
	pSensor->Open();
	/****************2.打开深度图像阅读器************************/
	// 取得深度数据
	IDepthFrameSource* pDepthSource = nullptr;
	pSensor->get_DepthFrameSource(&pDepthSource);
	// 取得深度数据的描述信息（宽、高）
	int        iDepthWidth = 0;
	int        iDepthHeight = 0;
	IFrameDescription* pDepthDescription = nullptr;
	pDepthSource->get_FrameDescription(&pDepthDescription);
	pDepthDescription->get_Width(&iDepthWidth);
	pDepthDescription->get_Height(&iDepthHeight);
	// 打开深度数据阅读器
	IDepthFrameReader* pDepthReader = nullptr;
	pDepthSource->OpenReader(&pDepthReader);
	pDepthDescription->Release();
	pDepthDescription = nullptr;
	// 释放变量pDepthSource
	pDepthSource->Release();
	pDepthSource = nullptr;

	/******************3.打开彩色图像阅读器**********************/
	// 取得彩色数据
	IColorFrameSource* pColorSource = nullptr;
	pSensor->get_ColorFrameSource(&pColorSource);
	// 取得彩色数据的描述信息（宽、高）
	int        iColorWidth = 0;
	int        iColorHeight = 0;
	IFrameDescription* pColorDescription = nullptr;
	pColorSource->get_FrameDescription(&pColorDescription);
	pColorDescription->get_Width(&iColorWidth);
	pColorDescription->get_Height(&iColorHeight);
	// 打开彩色数据阅读器
	IColorFrameReader* pColorReader = nullptr;
	pColorSource->OpenReader(&pColorReader);
	pColorDescription->Release();
	pColorDescription = nullptr;
	// 释放变量pColorSource
	pColorSource->Release();
	pColorSource = nullptr;

	/*******************4.打开人体索引的阅读器*******************/
	// 取得BodyIndex数据
	IBodyIndexFrameSource* pBodyIndexSource = nullptr;
	pSensor->get_BodyIndexFrameSource(&pBodyIndexSource);
	// 取得深度数据的描述信息（宽、高）
	int        iBodyIndexWidth = 0;
	int        iBodyIndexHeight = 0;
	IFrameDescription* pBodyIndexDescription = nullptr;
	pBodyIndexSource->get_FrameDescription(&pBodyIndexDescription);
	pBodyIndexDescription->get_Width(&iBodyIndexWidth);
	pBodyIndexDescription->get_Height(&iBodyIndexHeight);
	// 打开深度数据阅读器
	IBodyIndexFrameReader* pBodyIndexReader = nullptr;
	pBodyIndexSource->OpenReader(&pBodyIndexReader);
	pBodyIndexDescription->Release();
	pBodyIndexDescription = nullptr;
	// 释放变量pBodyIndexSource
	pBodyIndexSource->Release();
	pBodyIndexSource = nullptr;

	/*************5.为各种图像建立buffer，准备坐标转换*************/
	UINT    iColorDataSize = iColorHeight * iColorWidth;
	UINT    iDepthDataSize = iDepthHeight * iDepthWidth;
	UINT    iBodyIndexDataSize = iBodyIndexHeight * iBodyIndexWidth;
	//获取背景图并调整至彩色图像的大小
	Mat temp = imread("test.jpg"), background;
	resize(temp, background, Size(iColorWidth, iColorHeight));
	//开启mapper                                                          
	ICoordinateMapper   * myMaper = nullptr;
	pSensor->get_CoordinateMapper(&myMaper);
	//准备buffer
	Mat ColorData(iColorHeight, iColorWidth, CV_8UC4);
	UINT16  * DepthData = new UINT16[iDepthDataSize];
	BYTE    * BodyData = new BYTE[iBodyIndexDataSize];
	DepthSpacePoint * output = new DepthSpacePoint[iColorDataSize];

	/***********6.把各种图像读进buffer里，然后进行坐标转换以及替换**************/

	while (1)
	{
		//读取color图
		IColorFrame * pColorFrame = nullptr;
		while (pColorReader->AcquireLatestFrame(&pColorFrame) != S_OK);
		pColorFrame->CopyConvertedFrameDataToArray(iColorDataSize * 4, ColorData.data, ColorImageFormat_Bgra);
		pColorFrame->Release();

		//读取depth图
		IDepthFrame * pDepthframe = nullptr;
		while (pDepthReader->AcquireLatestFrame(&pDepthframe) != S_OK);
		pDepthframe->CopyFrameDataToArray(iDepthDataSize, DepthData);
		pDepthframe->Release();

		//读取BodyIndex图
		IBodyIndexFrame * pBodyIndexFrame = nullptr;
		while (pBodyIndexReader->AcquireLatestFrame(&pBodyIndexFrame) != S_OK);
		pBodyIndexFrame->CopyFrameDataToArray(iBodyIndexDataSize, BodyData);
		pBodyIndexFrame->Release();

		//复制一份背景图来做处理
		Mat copy = background.clone();
		if (myMaper->MapColorFrameToDepthSpace(iDepthDataSize, DepthData, iColorDataSize, output) == S_OK)
		{
			for (int i = 0; i < iColorHeight; ++i)
				for (int j = 0; j < iColorWidth; ++j)
				{
					//取得彩色图像上包含对应到深度图上的坐标的一点
					DepthSpacePoint tPoint = output[i * iColorWidth + j];
					//判断这个点是否合法
					if (tPoint.X >= 0 && tPoint.X < iDepthWidth && tPoint.Y >= 0 && tPoint.Y < iDepthHeight)
					{
						//取得彩色图上那点对应在BodyIndex里的值
						int index = (int)tPoint.Y * iDepthWidth + (int)tPoint.X;
						//判断出彩色图上某点是人体，就用替换背景图上对应点
						if (BodyData[index] <= 5)
						{
							Vec4b   color = ColorData.at<Vec4b>(i, j);
							copy.at<Vec3b>(i, j) = Vec3b(color[0], color[1], color[2]);
						}
					}
				}
			imshow("Background Remove", copy);
		}
		if (waitKey(30) == VK_ESCAPE) {
			break;
		}
	}
	delete[] DepthData;
	delete[] BodyData;
	delete[] output;

	// 1c.关闭感应器
	pSensor->Close();
	// 1d.释放感应器
	pSensor->Release();
	pSensor = nullptr;

	return 0;
}