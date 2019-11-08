#include "stdafx.h"
//�����ͼ��RGBͼ��ֻ�������岿��



#if 0
#include <opencv2\core\core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <Kinect.h>
#include <iostream>
//��ȡ���岿��
using namespace std;
using   namespace   cv;
int main(void)
{
	// 1a.��ȡ��Ӧ��
	IKinectSensor* pSensor = nullptr;
	GetDefaultKinectSensor(&pSensor);

	// 1b. �򿪸�Ӧ��
	pSensor->Open();

	// 2a. ȡ��BodyIndex����
	IBodyIndexFrameSource* pFrameSource = nullptr;
	pSensor->get_BodyIndexFrameSource(&pFrameSource);

	// 2b. ȡ��BodyIndex���ݵ�������Ϣ�����ߣ�
	int        iWidth = 0;
	int        iHeight = 0;
	IFrameDescription* pFrameDescription = nullptr;
	pFrameSource->get_FrameDescription(&pFrameDescription);
	pFrameDescription->get_Width(&iWidth);
	pFrameDescription->get_Height(&iHeight);
	pFrameDescription->Release();
	pFrameDescription = nullptr;

	// 3a. ��BodyIndex�����Ķ���
	IBodyIndexFrameReader* pFrameReader = nullptr;
	pFrameSource->OpenReader(&pFrameReader);

	IBodyIndexFrame* pFrame = nullptr;

	// ����ͼ�����mBodyIndexImg�����洢ͼ������
	Mat mBodyIndexImg(iHeight, iWidth, CV_8UC3);
	namedWindow("BodyIndexImage");

	// color array
	Vec3b   color[7] = { Vec3b(0,0,255),Vec3b(0,255,255),Vec3b(255,255,255),Vec3b(0,255,0),Vec3b(255,0,0),Vec3b(255,0,255),Vec3b(0,0,0) };

	// ��ѭ��
	while (1)
	{
		// 4a. ȡ����������
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			// 4c. �����ݴ���ͼ�������
			UINT uSize = 0;
			BYTE* pBuffer = nullptr;
			pFrame->AccessUnderlyingBuffer(&uSize, &pBuffer);
			for (int y = 0; y < iHeight; ++y)
			{
				for (int x = 0; x < iWidth; ++x)
				{
					int uBodyIdx = pBuffer[x + y * iWidth];//0-5�������壬����ֵ������
					if (uBodyIdx < 6)
						mBodyIndexImg.at<Vec3b>(y, x) = color[uBodyIdx];
					else
						mBodyIndexImg.at<Vec3b>(y, x) = color[6];
				}
			}
			imshow("Body Index Image", mBodyIndexImg);

			// 4e. �ͷű���pFrame
			pFrame->Release();
		}

		if (waitKey(30) == VK_ESCAPE) {
			break;
		}
	}

	// 3b. �ͷű���pFrameReader
	pFrameReader->Release();
	pFrameReader = nullptr;

	// 2c.�ͷű���pFramesSource
	pFrameSource->Release();
	pFrameSource = nullptr;

	// 1c.�رո�Ӧ��
	pSensor->Close();

	// 1d.�ͷŸ�Ӧ��
	pSensor->Release();
	pSensor = nullptr;

	return 0;
}
#endif




#if 0
#include <opencv2\core\core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <Kinect.h>
#include <iostream>
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
	//Mat temp(1080, 1920, CV_8UC4);
	//Mat temp(iColorWidth, iColorHeight CV_8UC4);
	Mat temp = imread("test.jpg"), background;
	cv::resize(temp, background, Size(iColorWidth, iColorHeight));
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
#endif

















#if 1
//��ȡ�����������RGBͼ�����ͼ512*424�ɹ�
#include <kinect.h>
#include <iostream>
#include <opencv2\opencv.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <kinect.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include<string>


//�����������

typedef pcl::PointXYZRGBA PointT;

typedef pcl::PointCloud<PointT> PointCloud;

using namespace cv;
using namespace std;

ICoordinateMapper* pMapper = nullptr;
const int iWidth = 512, iHeight = 424;
CameraSpacePoint depth2xyz[iWidth*iHeight];
ColorSpacePoint depth2rgb[iWidth*iHeight];
// ��ȫ�ͷ�ָ��
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


int countrgb = 1;
int countdepth = 1;

Mat ConvertMat_1(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	Mat img(nHeight, nWidth, CV_16UC1);
	UINT16* p_mat = (UINT16*)img.data;

	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		//USHORT depth = *pBuffer;
		*p_mat = *pBuffer;
		//*p_mat = (depth >= nMinDepth) && (depth <= nMaxDepth) ? depth  : 0;
		//*p_mat = (depth >= nMinDepth) && (depth <= nMaxDepth) ? depth >> 0 : 0;
		p_mat++;
		++pBuffer;
	}
	return img;
}


int GetPicture()
{


	// ��ȡKinect�豸
	IKinectSensor* m_pKinectSensor;
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	IMultiSourceFrameReader* m_pMultiFrameReader;
	if (m_pKinectSensor)
	{
		m_pKinectSensor->get_CoordinateMapper(&pMapper);
		hr = m_pKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			// ��ȡ������Դ����ȡ��  
			/*hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Infrared |
				FrameSourceTypes::FrameSourceTypes_Depth,
				&m_pMultiFrameReader);*/
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Infrared |
				FrameSourceTypes::FrameSourceTypes_Depth|
				FrameSourceTypes::FrameSourceTypes_BodyIndex,
				&m_pMultiFrameReader);
		}


	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}

	UINT    iBodyIndexDataSize = iHeight * iWidth;
	BYTE    * BodyData = new BYTE[iBodyIndexDataSize];



	// ��������֡������
	IDepthFrameReference* m_pDepthFrameReference;
	IColorFrameReference* m_pColorFrameReference;
	IInfraredFrameReference* m_pInfraredFrameReference;
	IBodyIndexFrameReference * m_pBodyIndexFrameReference;
	//IBodyFrameReference* m_pBodyFrameReference;
	IInfraredFrame* m_pInfraredFrame;
	IDepthFrame* m_pDepthFrame;
	IColorFrame* m_pColorFrame;
	IBodyIndexFrame * m_pBodyIndexFrame;
	//IBodyFrame* m_pBodyFrame;
	// ȡ��BodyIndex����
	//IBodyIndexFrameSource* pBodyIndexSource = nullptr;
	// ����ͼƬ��ʽ
	Mat i_rgb(1080, 1920, CV_8UC4);      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
	Mat i_rgb_3channel;
	Mat i_depth(424, 512, CV_16UC1);
	Mat i_depth_onlyman(424, 512, CV_16UC1);
	//Mat i_depth_32bir(424, 512, CV_32S);
	//Mat i_depth(424, 512, CV_16UC1);
	//Mat i_rgb(480,640, CV_8UC4);      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
	//Mat i_depth(480, 640, CV_8UC1);
	Mat i_ir(424, 512, CV_16UC1);

	UINT16 *depthData = new UINT16[424 * 512];
	//INT32 *depthData_32bit = new INT32[424 * 512];
	IMultiSourceFrame* m_pMultiFrame = nullptr;
	int sample_id = 1;






	while (true)
	{
		// ��ȡ�µ�һ����Դ����֡

		hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
		if (FAILED(hr) || !m_pMultiFrame)
		{
			cout << "!!!" << endl;
			continue;
		}


		// �Ӷ�Դ����֡�з������ɫ���ݣ�������ݺͺ�������
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_ColorFrameReference(&m_pColorFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pColorFrameReference->AcquireFrame(&m_pColorFrame);
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_DepthFrameReference(&m_pDepthFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pDepthFrameReference->AcquireFrame(&m_pDepthFrame);
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_InfraredFrameReference(&m_pInfraredFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pInfraredFrameReference->AcquireFrame(&m_pInfraredFrame);


		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_BodyIndexFrameReference(&m_pBodyIndexFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pBodyIndexFrameReference->AcquireFrame(&m_pBodyIndexFrame);
		

		// color������ͼƬ��
		UINT nColorBufferSize = 1920 * 1080 * 4;
		//UINT nColorBufferSize = 640 * 480 * 4;
		if (SUCCEEDED(hr))
			hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);
		Mat i_depthToRgb(424, 512, CV_8UC4);
		//Mat i_depthToRgb_onlyman(424, 512, CV_8UC4);
		Mat i_depthToRgb_onlyman = imread("C:\\vsprojects\\test\\res\\test.png");
		resize(i_depthToRgb_onlyman, i_depthToRgb_onlyman, Size(512, 424));

		i_rgb_3channel = i_rgb;
		//cv::imwrite("C:/vsprojects/test/rgb.png", i_rgb_3channel);


		// depth������ͼƬ��
		//i_depthToRgb�Ǻ����ͼ�����512*424 RGBͼ��
		if (SUCCEEDED(hr))
		{
			//����yqy
			hr = pMapper->MapDepthFrameToColorSpace(512 * 424, depthData, 512 * 424, depth2rgb);
			for (int i = 0; i < 424 * 512; i++)
			{
				ColorSpacePoint p = depth2rgb[i];
				if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
				{
					int colorX = static_cast<int>(p.X + 0.5f);
					int colorY = static_cast<int>(p.Y + 0.5f);

					if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
					{
						i_depthToRgb.data[i * 4] = i_rgb.data[(colorY * 1920 + colorX) * 4];
						i_depthToRgb.data[i * 4 + 1] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 1];
						i_depthToRgb.data[i * 4 + 2] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 2];
						i_depthToRgb.data[i * 4 + 3] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 3];
					}
				}
			}
			//yqyend
			USHORT nDepthMinReliableDistance = 0;//��ȡ�����С��Ⱦ�����Ϣ
			USHORT nDepthMaxReliableDistance = 0;
			assert(hr >= 0);
			hr = m_pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			assert(hr >= 0);
			hr = m_pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);

			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);

			i_depth = ConvertMat_1(depthData, 512, 424, nDepthMinReliableDistance, nDepthMaxReliableDistance);
			//hr = m_pDepthFrame->CopyFrameDataToArray(480 * 640, depthData);
			//for (int i = 0; i < 512 * 424; i++)
			//	//for (int i = 0; i < 640 * 480; i++)
			//{
			//	// 0-255���ͼ��Ϊ����ʾ���ԣ�ֻȡ������ݵĵ�8λ
			//	BYTE intensity = static_cast<BYTE>(depthData[i] % 65536);
			//	//BYTE intensity = static_cast<BYTE>(depthData[i] % 256);
			//	reinterpret_cast<BYTE*>(i_depth.data)[i] = intensity;
			//}

			// ʵ����16λunsigned int����
			//hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth.data));
		}

		// infrared������ͼƬ��
		if (SUCCEEDED(hr))
		{
			hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_ir.data));
			//hr = m_pInfraredFrame->CopyFrameDataToArray(480 * 640, reinterpret_cast<UINT16*>(i_ir.data));
		}


		if (SUCCEEDED(hr))
		{
			m_pBodyIndexFrame->CopyFrameDataToArray(iBodyIndexDataSize, BodyData);

			for (int i = 0; i < 424; ++i)
				for (int j = 0; j < 512; ++j)
				{
					int index = i * 512 + j;
					//�жϳ���ɫͼ��ĳ�������壬�����滻����ͼ�϶�Ӧ��
					if (BodyData[index] <= 5)
					{
						Vec4b  color = i_depthToRgb.at<Vec4b>(i, j);
						i_depthToRgb_onlyman.at<Vec3b>(i, j) = Vec3b(color[0], color[1], color[2]);
						i_depth_onlyman.at<ushort>(i, j) = i_depth.at<ushort>(i, j);
						//i_depthToRgb_onlyman.data[index] = i_depthToRgb.data[index];
						//i_depth_onlyman.data[index] = i_depth.data[index];
						//i_depth_onlyman.at<Vec>(i, j) = i_depth.ptr<uchar>(index);
							/*Vec depth = i_depth.at<Vec>(i, j);
						i_depth_onlyman.at<Vec>(i, j) = (depth[0]);*/
					}
					else
					{
						i_depthToRgb_onlyman.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
						i_depth_onlyman.at<ushort>(i, j) = 0;
					}

				}
		}
			

		// ��ʾ
		imshow("rgb", i_depthToRgb_onlyman);
		std::stringstream str1;
		if (countrgb < 50)
		{
			str1 << "C:/vsprojects/test/res/rgbonlyman/" << countrgb << ".png";
			countrgb++;
			imwrite(str1.str(), i_depthToRgb_onlyman);
		}
		//std::cout << str1.str();

		//imwrite("C:/vsprojects/test/res/rgb/2.png", i_depthToRgb);


		if (waitKey(1) == VK_ESCAPE)
			break;
		//cv::equalizeHist(i_depth, i_depth);//���⻯��Ϊ�������ʾЧ��
		imshow("depth", i_depth_onlyman);
		//Mat img32BIT;
		////float scaleFactor = 1.0; // Or what you want 
		//i_depth.convertTo(img32BIT, CV_32S, 1.0 / 65535.0f);

		std::stringstream str2;
		if (countdepth <50)
		{
			str2 << "C:/vsprojects/test/res/depthonlyman/" << countdepth << ".png";
			countdepth++;
			imwrite(str2.str(), i_depth_onlyman);
		}


		/*std::stringstream str3;
		if (countdepth <50)
		{
			str3 << "C:/vsprojects/test/res/idepth/" << countdepth << ".png";
			countdepth++;
			imwrite(str3.str(), i_depth);
		}*/

		if (cv::waitKey(30) == VK_ESCAPE) break;


		// �ͷ���Դ
		SafeRelease(m_pColorFrame);
		SafeRelease(m_pDepthFrame);
		SafeRelease(m_pInfraredFrame);
		SafeRelease(m_pBodyIndexFrame);//add yqy
		SafeRelease(m_pColorFrameReference);
		SafeRelease(m_pDepthFrameReference);
		SafeRelease(m_pInfraredFrameReference);
		SafeRelease(m_pBodyIndexFrameReference);//add yqy
		SafeRelease(m_pMultiFrame);
	}
	// �رմ��ڣ��豸
	cv::destroyAllWindows();
	m_pKinectSensor->Close();
}
int main()
{
	GetPicture();
	std::system("pause");
	return 0;
}
#endif