//
// pcd2binvox
// Convert a .pcd file to .binvox
//
// pcd is Point Cloud Data from PCL (PointCloud Library).
// binvox is a binary format for a 3D voxel grid.
//
// David Butterworth, 2016.
//
// binvox was developed by Patrick Min (www.patrickmin.com)
// The RLE code below is based on binvox-rw-py by Daniel Maturana.



//将pcd转为binvox体素文件（带纹理，纹理存文texture.txt文件）
//在项目-属性-配置属性-命令参数中输入test  -d 1024  ./xiaowenstatisticalOutlierRemoval_inlier190820.pcd ./xiaowenstatisticalOutlierRemoval_inlier190901_texture.binvox

#include "stdafx.h"
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip> // setprecision

#include <cstdlib>
#include <cstring>

#include <boost/dynamic_bitset.hpp>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

#include <vector>
#if 0
using namespace std;

//struct Voxel
//{
//	unsigned int x;
//	unsigned int y;
//	unsigned int z;
//	Voxel() : x(0), y(0), z(0) {};
//	Voxel(const unsigned int _x, const unsigned int _y, const unsigned int _z) : x(_x), y(_y), z(_z) {};
//};


struct Voxel
{
	unsigned int x;
	unsigned int y;
	unsigned int z;
	unsigned int r;
	unsigned int g;
	unsigned int b;
	Voxel() : x(0), y(0), z(0),r(0), g(0), b(0) {};
	Voxel(const unsigned int _x, const unsigned int _y, const unsigned int _z, const unsigned int _r, const unsigned int _g, const unsigned int _b) : x(_x), y(_y), z(_z), r(_r), g(_g), b(_b) {};
};


/*
// For debugging: Write the voxel indices to a file
void writeVoxelsToFile(const std::vector<Voxel>& voxels, const std::string& filename)
{
std::ofstream* output = new std::ofstream(filename, std::ios::out);
if (!output->good())
{
std::cerr << "Error: Could not open output file " << output << "! \n" << std::endl;
exit(1);
}
for (size_t i = 0; i < voxels.size(); ++i)
{
// Write in order (X,Z,Y)
*output << voxels.at(i).x << " "  << voxels.at(i).z << " "  << voxels.at(i).y << "\n";
}
output->close();
}
*/

template <typename PointT>
const bool loadPointCloud(const std::string& file_path,
	typename pcl::PointCloud<PointT>& cloud_out)
{
	//if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path.c_str(), cloud_out) == -1)
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_path.c_str(), cloud_out) == -1)
	{
		PCL_ERROR("Failed to load PCD file \n");
		return false;
	}

	return true;
}

// Get the linear index into a 1D array of voxels,
// for a voxel at location (ix, iy, iz).
const unsigned int getLinearIndex(const Voxel& voxel, const int grid_size)
{
	return voxel.x * (grid_size * grid_size) + voxel.z * grid_size + voxel.y;
}

const Voxel getGridIndex(const pcl::PointXYZRGB& point, const pcl::PointXYZ& translate, const unsigned int voxel_grid_size, const float scale)
//const Voxel getGridIndex(const pcl::PointXYZRGB& point, const pcl::PointXYZRGB& translate, const unsigned int voxel_grid_size, const float scale)
//const Voxel getGridIndex(const pcl::PointXYZ& point, const pcl::PointXYZ& translate, const unsigned int voxel_grid_size, const float scale)
//const Voxel getGridIndex(const pcl::PointXYZ& point, const pcl::PointXYZ& translate, const uint voxel_grid_size, const float scale)
{
	// Needs to be signed to prevent overflow, because index can
	// be slightly negative which then gets rounded to zero.
	const int i = std::round(static_cast<float>(voxel_grid_size)*((point.x - translate.x) / scale) - 0.5);
	const int j = std::round(static_cast<float>(voxel_grid_size)*((point.y - translate.y) / scale) - 0.5);
	const int k = std::round(static_cast<float>(voxel_grid_size)*((point.z - translate.z) / scale) - 0.5);


	const int r = point.r;
	const int g = point.g;
	const int b = point.b;
	//return Voxel(i, j, k);
	return Voxel(i, j, k,r,g,b);
}

// Format a float number to ensure it always has at least one decimal place
// 0 --> 0.0
// 1.1 --> 1.1
// 1.10 --> 1.1
const std::string formatFloat(const float value)
{
	std::stringstream ss;
	ss << std::setprecision(6) << std::fixed << value;
	std::string str;
	ss.str().swap(str);
	size_t last_zero_idx = str.find_last_not_of("0") + 1;
	if (last_zero_idx == str.length())
	{
		// No trailing zeros
		return str;
	}
	if (str[last_zero_idx - 1] == '.')
	{
		// Last zero is after decimal point
		last_zero_idx += 1;
	}
	str.resize(last_zero_idx);
	return str;
}



//在项目-属性-配置属性-命令参数中输入test  -d 1024  ./xiaowenstatisticalOutlierRemoval_inlier190820.pcd ./xiaowenstatisticalOutlierRemoval_inlier190820.binvox
int main(int argc, char **argv)
{
	if (argc < 4)
	{
		pcl::console::print_error("Syntax is: %s -d <voxel_grid_size [32 to 1024]> input.pcd output.binvox \n", argv[0]);
		return -1;
	}

	// Parse the command line arguments for .pcd and .ply files
	std::vector<int> pcd_file_indices = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
	std::vector<int> binvox_file_indices = pcl::console::parse_file_extension_argument(argc, argv, ".binvox");
	if (pcd_file_indices.size() != 1 || binvox_file_indices.size() != 1)
	{
		pcl::console::print_error("Need one input PCD file and one output Binvox file. \n");
		return -1;
	}

	// In binvox, default is 256, max 1024
	//uint voxel_grid_size;
	unsigned int voxel_grid_size;
	pcl::console::parse_argument(argc, argv, "-d", voxel_grid_size);
	std::cout << "Voxel grid size: " << voxel_grid_size << std::endl;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	const std::string input_file(argv[pcd_file_indices[0]]);
	//if (!loadPointCloud<pcl::PointXYZ>(input_file, *cloud))
	if (!loadPointCloud<pcl::PointXYZRGB>(input_file, *cloud))
	{
		return -1;
	}

	/*pcl::PointXYZ min_point;
	pcl::PointXYZ max_point;*/
	pcl::PointXYZRGB min_point;
	pcl::PointXYZRGB max_point;
	pcl::getMinMax3D(*cloud, min_point, max_point);//即cloud为输入点云，而非指针（共享指针则写为*cloud），输出min_pt为所有点中最小的x值，y值，z值，输出max_pt为为所有点中最大的x值，y值，z值。

	// Calculate the scale factor so the longest side of the volume
	// is split into the desired number of voxels
	const float x_range = max_point.x - min_point.x;
	const float y_range = max_point.y - min_point.y;
	const float z_range = max_point.z - min_point.z;

	const float max_cloud_extent = std::max(std::max(x_range, y_range), z_range);
	const float voxel_size = max_cloud_extent / (static_cast<float>(voxel_grid_size) - 1.0);
	std::cout << "voxel_size = " << voxel_size << std::endl;

	const float scale = (static_cast<float>(voxel_grid_size) * max_cloud_extent) / (static_cast<float>(voxel_grid_size) - 1.0);

	std::cout << "Bounding box: "
		<< "[" << min_point.x << ", " << min_point.y << ", " << min_point.z << "] - "
		<< "[" << max_point.x << ", " << max_point.y << ", " << max_point.z << "]" << std::endl;

	// Calculate the PointCloud's translation from the origin.
	// We need to subtract half the voxel size, because points
	// are located in the center of the voxel grid. 
	float tx = min_point.x - voxel_size / 2.0;
	float ty = min_point.y - voxel_size / 2.0;
	float tz = min_point.z - voxel_size / 2.0;


	//float tr = min_point.r ;
	//float tg = min_point.g ;
	//float tb = min_point.b ;


	// Hack, change -0.0 to 0.0
	const float epsilon = 0.0000001;
	if ((tx > -epsilon) && (tx < 0.0))
	{
		tx = -1.0 * tx;
	}
	if ((ty > -epsilon) && (ty < 0.0))
	{
		ty = -1.0 * ty;
	}
	if ((tz > -epsilon) && (tz < 0.0))
	{
		tz = -1.0 * tz;
	}
	const pcl::PointXYZ translate(tx, ty, tz);
	//const pcl::PointXYZRGB translate(tx, ty, tz);
	std::cout << "Normalization transform: (1) translate ["
		<< formatFloat(translate.x) << ", " << formatFloat(translate.y) << ", " << formatFloat(translate.z) << "], " << std::endl;
	std::cout << "                         (2) scale " << scale << std::endl;

	const unsigned int num_voxels = voxel_grid_size * voxel_grid_size * voxel_grid_size;//comment yqy
	
	//const  unsigned long long num_voxels = 4.0* voxel_grid_size * voxel_grid_size * voxel_grid_size;
	//add yqy  unsigned int最大表示4294967295，乘以4太大溢出了就变成了0,所以要改成long long类型,并且要乘4.0不能乘4;还是不行，dynamic_bitset內部存儲的二進制位表示不能超過unsigned long的最大值，所以这边能表示后面也会溢出

	// Voxelize the PointCloud into a linear array
	//boost::dynamic_bitset<> voxels_bitset(num_voxels);//comment yqy
	
	//add yqy
	boost::dynamic_bitset<> voxels_bitsetpos(num_voxels);
	//boost::dynamic_bitset<> voxels_bitsetr(num_voxels);//不能这样写，boost::dynamic_bitset<> 是存储二进制位的，只有0和1
	//boost::dynamic_bitset<> voxels_bitsetg(num_voxels);
	//boost::dynamic_bitset<> voxels_bitsetb(num_voxels);



	std::vector<int> voxels_bitsetr(num_voxels);
	std::vector<int> voxels_bitsetg(num_voxels);
	std::vector<int> voxels_bitsetb(num_voxels);
	std::vector<std::vector<int>> voxels_bitset;
	//for (int i = 0; i < 3; ++i)
	{
		//voxels_bitset.push_back(voxels_bitsetpos);
		voxels_bitset.push_back(voxels_bitsetr);
		voxels_bitset.push_back(voxels_bitsetg);
		voxels_bitset.push_back(voxels_bitsetb);
	}
	//add end

	//boost::dynamic_bitset<> voxels_bitset(4*num_voxels);

	//add yqy
	/*boost::dynamic_bitset<> voxels_bitsetr(num_voxels);
	boost::dynamic_bitset<> voxels_bitsetg(num_voxels);
	boost::dynamic_bitset<> voxels_bitsetb(num_voxels);*/
	//yqy end

	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it != cloud->end(); ++it)
	//for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it)
	{
		const Voxel voxel = getGridIndex(*it, translate, voxel_grid_size, scale);
		const unsigned int idx = getLinearIndex(voxel, voxel_grid_size);
		//voxels_bitset[idx] = 1;
		//add yqy
		/*voxels_bitset[idx] = 1;
		voxels_bitset[idx+ num_voxels] = voxel.r;
		voxels_bitset[idx +2* num_voxels] = voxel.g;
		voxels_bitset[idx +3* num_voxels] = voxel.b;*/

		//voxels_bitset[0][idx] = 1;
		voxels_bitsetpos[idx] = 1;
		voxels_bitset[0][idx] = voxel.r;//不行，不能这样表示，位置可以是有点云为1，无点云为0；rgb值本来就可能有为0的，无法分辨
	/*	std::cout << "voxel.r" << voxel.r << std::endl;
		std::cout << "voxels_bitset" << voxels_bitset[0][idx] << std::endl;*/
		voxels_bitset[1][idx] = voxel.g;
		voxels_bitset[2][idx] = voxel.b;
	/*	voxels_bitsetr[idx] = voxel.r;
		voxels_bitsetg[idx] = voxel.g;
		voxels_bitsetb[idx] = voxel.b;*/
		//yqy end
	}

	/*
	// For debugging: Write voxel indices to a file
	std::vector<Voxel> voxels; // for debugging
	for (size_t i = 0; i < voxels_bitset.size(); ++i)
	{
	if (voxels_bitset[i] == 1)
	{
	const int voxel_grid_width = voxel_grid_size;
	const int voxel_grid_height = voxel_grid_size;
	const int idx = static_cast<int>(i);
	const float ix = static_cast<float>(idx / (voxel_grid_width * voxel_grid_height));
	const float iy = static_cast<float>(idx % voxel_grid_size);
	const float iz = static_cast<float>((idx / voxel_grid_width) % voxel_grid_height);
	voxels.push_back( Voxel(ix, iy, iz) );
	}
	}
	writeVoxelsToFile(voxels, "voxels_from_pcd.txt");
	*/

	const std::string output_file(argv[binvox_file_indices[0]]);
	std::ofstream* output = new std::ofstream(output_file, std::ios::out | std::ios::binary);
	//std::ofstream* output = new std::ofstream(output_file);//改成这样会出错滴
	if (!output->good())
	{
		std::cerr << "Error: Could not open output file " << output << "!" << std::endl;
		exit(1);
	}


	//新建一个文件存纹理？

	std::ofstream outtxt;
	outtxt.open("texture.txt");
	//std::ofstream outtxt("texture.txt", std::ios::binary);//使用二进制方法存，因为pcl点云rgb值赋值是二进制的
	//std::ofstream* outtxt = new std::ofstream("texture.txt", std::ios::out | std::ios::binary);

	// Write the binvox file using run-length encoding
	// where each pair of bytes is of the format (run value, run length)
	*output << "#binvox 1\n";
	*output << "dim " << voxel_grid_size << " " << voxel_grid_size << " " << voxel_grid_size << "\n";
	*output << "translate " << formatFloat(translate.x) << " " << formatFloat(translate.y) << " " << formatFloat(translate.z) << "\n";
	*output << "scale " << scale << "\n";
	*output << "data\n";

	////yqy add如果要添加纹理，那么rgb值几乎不可能相同，使用游程编码反而会增加内存，所以不用游程编码，就直接按顺序存(结论不对，事实表明这样一个文件要几个g),还是使用游程编码只要30M
	//for (size_t i = 0; i < num_voxels; ++i)
	//{
	//	*output << static_cast<char>(voxels_bitset[i]);
	//	*output << static_cast<char>(voxels_bitsetr[i]);
	//	*output << static_cast<char>(voxels_bitsetg[i]);
	//	*output << static_cast<char>(voxels_bitsetb[i]);
	//}
	////yqy end


	//yqy comment
	//游程编码，一组数据串"AAAABBBCCDEEEE"，由4个A、3个B、2个C、1个D、4个E组成，经过变动长度编码法可将数据压缩为4A3B2C1D4E
	//unsigned int run_value = voxels_bitset[0];//comment yqy
	unsigned int run_value = voxels_bitsetpos[0];//add yqy
	unsigned int run_length = 0;
	//for (size_t i = 0; i < 4*num_voxels; ++i)
	//先存有点的位置表示1，存完所有1再存r，存完所有r再存g，存完所有g再存b
	//试试存完每个位置点云的有无，if 有，则赋给rgb值
	//for (int j = 0; j < 4; ++j)
	//int j = 0;
	//先使用一个暂存函数Q读取第一个数据，接着将下一个数据与Q值比，若数据相同，则计数器加1；若数据不同，则将计数器存的数值以及Q值输出，再初始计数器为，Q值改为下一个数据。以此类推，完成数据压缩。
	//{
		//if (voxels_bitset[i] == run_value)//comment yqy
		for (size_t i = 0; i < num_voxels; ++i)
		{
			if (voxels_bitsetpos[i] == run_value)
			{
				// This is a run (repeated bit value)
				run_length++;
				if (run_length == 255)//8位表示法，其能表示最大整数为255。假设某数据出现N次，则可以将其分成(N/255)+1段落来处理
				{
					/*	outtxt<< run_value;
					outtxt << run_length;*/

					*output << static_cast<char>(run_value);
					*output << static_cast<char>(run_length);
					run_length = 0;
				}
			}
			else
			{
				// End of a run

				/*outtxt << (run_value);
				outtxt << (run_length);*/


				*output << static_cast<char>(run_value);
				*output << static_cast<char>(run_length);
				//run_value = voxels_bitset[i];//comment yqy
				run_value = voxels_bitsetpos[i];				
				//run_value = voxels_bitset[j][i];//add yqy
				run_length = 1;
			}


			
			
			

		}

		//add yqy 存完每个位置点云的有无后，if 有，则赋给rgb值
		unsigned int countallpoint = 0;
		for (size_t i = 0; i < num_voxels; ++i)
		{
			if (voxels_bitsetpos[i] == 1)
			{
				countallpoint += 1;
				/**output << voxels_bitset[1][i];
				*output << voxels_bitset[2][i];
				*output << voxels_bitset[3][i];*/
				outtxt << voxels_bitset[0][i]<<" ";
				//std::cout<< voxels_bitset[1][i];
				outtxt << voxels_bitset[1][i] << " ";
				outtxt << voxels_bitset[2][i] << " ";



				//*outtxt << voxels_bitset[0][i] << " ";
				////std::cout<< voxels_bitset[1][i];
				//*outtxt << voxels_bitset[1][i] << " ";
				//*outtxt << voxels_bitset[2][i] << " ";

				////二进制方式
				//outtxt->write(reinterpret_cast<char*>(&voxels_bitset[0][i]), sizeof(voxels_bitset[0][i]));
				//outtxt->write(reinterpret_cast<char*>(&voxels_bitset[1][i]), sizeof(voxels_bitset[1][i]));
				//outtxt->write(reinterpret_cast<char*>(&voxels_bitset[2][i]), sizeof(voxels_bitset[2][i]));
			}
		}
		std::cout << "countallpoint " << countallpoint << std::endl;//统计有多少个点，方便后面voxel2pcd读取
		//yqy end



	//}
	if (run_length > 0)
	{
		/*outtxt << (run_value);
		outtxt << (run_length);*/


		*output << static_cast<char>(run_value);
		*output << static_cast<char>(run_length);
	}
	//comment end
	
	output->close();
	//outtxt->close();//add yqy
	outtxt.close();//add yqy


	



	std::cout << "done" << std::endl << std::endl;
	return 0;
}
#endif





