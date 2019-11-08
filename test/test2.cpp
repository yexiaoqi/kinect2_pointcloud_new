#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Kinect.h>

using namespace std;
using namespace cv;

int main(void)
{
	// 1a.��ȡ��Ӧ��
	IKinectSensor* pSensor = nullptr;
	GetDefaultKinectSensor(&pSensor);
	// 1b. �򿪸�Ӧ��
	pSensor->Open();
	/****************2.�����ͼ���Ķ���************************/
	// ȡ���������
	IDepthFrameSource* pDepthSource = nullptr;
	pSensor->get_DepthFrameSource(&pDepthSource);
	// ȡ��������ݵ�������Ϣ�����ߣ�
	int        iDepthWidth = 0;
	int        iDepthHeight = 0;
	IFrameDescription* pDepthDescription = nullptr;
	pDepthSource->get_FrameDescription(&pDepthDescription);
	pDepthDescription->get_Width(&iDepthWidth);
	pDepthDescription->get_Height(&iDepthHeight);
	// ����������Ķ���
	IDepthFrameReader* pDepthReader = nullptr;
	pDepthSource->OpenReader(&pDepthReader);
	pDepthDescription->Release();
	pDepthDescription = nullptr;
	// �ͷű���pDepthSource
	pDepthSource->Release();
	pDepthSource = nullptr;

	/******************3.�򿪲�ɫͼ���Ķ���**********************/
	// ȡ�ò�ɫ����
	IColorFrameSource* pColorSource = nullptr;
	pSensor->get_ColorFrameSource(&pColorSource);
	// ȡ�ò�ɫ���ݵ�������Ϣ�����ߣ�
	int        iColorWidth = 0;
	int        iColorHeight = 0;
	IFrameDescription* pColorDescription = nullptr;
	pColorSource->get_FrameDescription(&pColorDescription);
	pColorDescription->get_Width(&iColorWidth);
	pColorDescription->get_Height(&iColorHeight);
	// �򿪲�ɫ�����Ķ���
	IColorFrameReader* pColorReader = nullptr;
	pColorSource->OpenReader(&pColorReader);
	pColorDescription->Release();
	pColorDescription = nullptr;
	// �ͷű���pColorSource
	pColorSource->Release();
	pColorSource = nullptr;

	/*******************4.�������������Ķ���*******************/
	// ȡ��BodyIndex����
	IBodyIndexFrameSource* pBodyIndexSource = nullptr;
	pSensor->get_BodyIndexFrameSource(&pBodyIndexSource);
	// ȡ��������ݵ�������Ϣ�����ߣ�
	int        iBodyIndexWidth = 0;
	int        iBodyIndexHeight = 0;
	IFrameDescription* pBodyIndexDescription = nullptr;
	pBodyIndexSource->get_FrameDescription(&pBodyIndexDescription);
	pBodyIndexDescription->get_Width(&iBodyIndexWidth);
	pBodyIndexDescription->get_Height(&iBodyIndexHeight);
	// ����������Ķ���
	IBodyIndexFrameReader* pBodyIndexReader = nullptr;
	pBodyIndexSource->OpenReader(&pBodyIndexReader);
	pBodyIndexDescription->Release();
	pBodyIndexDescription = nullptr;
	// �ͷű���pBodyIndexSource
	pBodyIndexSource->Release();
	pBodyIndexSource = nullptr;

	/*************5.Ϊ����ͼ����buffer��׼������ת��*************/
	UINT    iColorDataSize = iColorHeight * iColorWidth;
	UINT    iDepthDataSize = iDepthHeight * iDepthWidth;
	UINT    iBodyIndexDataSize = iBodyIndexHeight * iBodyIndexWidth;
	//��ȡ����ͼ����������ɫͼ��Ĵ�С
	Mat temp = imread("test.jpg"), background;
	resize(temp, background, Size(iColorWidth, iColorHeight));
	//����mapper                                                          
	ICoordinateMapper   * myMaper = nullptr;
	pSensor->get_CoordinateMapper(&myMaper);
	//׼��buffer
	Mat ColorData(iColorHeight, iColorWidth, CV_8UC4);
	UINT16  * DepthData = new UINT16[iDepthDataSize];
	BYTE    * BodyData = new BYTE[iBodyIndexDataSize];
	DepthSpacePoint * output = new DepthSpacePoint[iColorDataSize];

	/***********6.�Ѹ���ͼ�����buffer�Ȼ���������ת���Լ��滻**************/

	while (1)
	{
		//��ȡcolorͼ
		IColorFrame * pColorFrame = nullptr;
		while (pColorReader->AcquireLatestFrame(&pColorFrame) != S_OK);
		pColorFrame->CopyConvertedFrameDataToArray(iColorDataSize * 4, ColorData.data, ColorImageFormat_Bgra);
		pColorFrame->Release();

		//��ȡdepthͼ
		IDepthFrame * pDepthframe = nullptr;
		while (pDepthReader->AcquireLatestFrame(&pDepthframe) != S_OK);
		pDepthframe->CopyFrameDataToArray(iDepthDataSize, DepthData);
		pDepthframe->Release();

		//��ȡBodyIndexͼ
		IBodyIndexFrame * pBodyIndexFrame = nullptr;
		while (pBodyIndexReader->AcquireLatestFrame(&pBodyIndexFrame) != S_OK);
		pBodyIndexFrame->CopyFrameDataToArray(iBodyIndexDataSize, BodyData);
		pBodyIndexFrame->Release();

		//����һ�ݱ���ͼ��������
		Mat copy = background.clone();
		if (myMaper->MapColorFrameToDepthSpace(iDepthDataSize, DepthData, iColorDataSize, output) == S_OK)
		{
			for (int i = 0; i < iColorHeight; ++i)
				for (int j = 0; j < iColorWidth; ++j)
				{
					//ȡ�ò�ɫͼ���ϰ�����Ӧ�����ͼ�ϵ������һ��
					DepthSpacePoint tPoint = output[i * iColorWidth + j];
					//�ж�������Ƿ�Ϸ�
					if (tPoint.X >= 0 && tPoint.X < iDepthWidth && tPoint.Y >= 0 && tPoint.Y < iDepthHeight)
					{
						//ȡ�ò�ɫͼ���ǵ��Ӧ��BodyIndex���ֵ
						int index = (int)tPoint.Y * iDepthWidth + (int)tPoint.X;
						//�жϳ���ɫͼ��ĳ�������壬�����滻����ͼ�϶�Ӧ��
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

	// 1c.�رո�Ӧ��
	pSensor->Close();
	// 1d.�ͷŸ�Ӧ��
	pSensor->Release();
	pSensor = nullptr;

	return 0;
}