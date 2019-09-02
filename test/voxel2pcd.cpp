#include "stdafx.h"


//将binvox体素文件转为pcd
//在项目-属性-配置属性-命令参数中输入 -o  ./xiaowenstatisticalOutlierRemoval_inlier190901_texture.pcd ./xiaowenstatisticalOutlierRemoval_inlier190901_texture.binvox


//
// binvox2pcd
// Convert a .binvox file to .pcd
//
// binvox is a binary format for a 3D voxel grid.
// pcd is Point Cloud Data from PCL (PointCloud Library).
//
// David Butterworth, 2016.
//
// binvox was developed by Patrick Min (www.patrickmin.com)
// The code below is based on Patrick's read_binvox.cc
// and binvox2bt.cpp by Stefan Osswald.
//

#include <string>
#include <fstream>
#include <iostream>

#include <cstdlib>
#include <cstring>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include<vector>

#include <string>


// TODO: - Test input of multiple binvox files
//       - Use boost to check for missing file extension


/*
struct Voxel
{
unsigned int x;
unsigned int y;
unsigned int z;
Voxel() : x(0), y(0), z(0) {};
Voxel(const unsigned int _x, const unsigned int _y, const unsigned int _z) : x(_x), y(_y), z(_z) {};
};
// For debugging: Write the voxel indices to a file
void writeVoxelsToFile(const std::vector<Voxel>& voxels, const std::string& filename)
{
std::ofstream* output = new std::ofstream(filename, std::ios::out);
if (!output->good())
{
std::cerr << "Error: Could not open output file " << output << "!" << std::endl;
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
#if 0
//在项目-属性-配置属性-命令参数中输入 -o  ./xiaowenstatisticalOutlierRemoval_inlier190901_texture.pcd ./xiaowenstatisticalOutlierRemoval_inlier190901_texture.binvox

int main(int argc, char **argv)
{
	bool show_help = false;
	if (argc == 1)
	{
		show_help = true;
	}
	for (int i = 1; i < argc && !show_help; ++i)
	{
		if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ||
			strcmp(argv[i], "--usage") == 0 || strcmp(argv[i], "-usage") == 0 ||
			strcmp(argv[i], "-h") == 0)
		{
			show_help = true;
		}
	}
	if (show_help)
	{
		std::cout << "Usage: " << argv[0] << " [OPTIONS] <binvox filenames>" << std::endl;
		std::cout << "\tOPTIONS:" << std::endl;
		std::cout << "\t -o <file>        Output filename (default: first input filename + .pcd)\n";
		std::cout << "\t --mark-free      Mark not occupied cells as 'free' (default: unknown)\n";
		std::cout << "\t --bb <bbox_min_x> <bbox_min_y> <bbox_min_z> <bbox_max_x> <bbox_max_y> <bbox_max_z>: enforce bounding box\n";
		std::cout << "All options apply to the subsequent input files.\n\n";
		exit(0);
	}

	std::string output_filename;
	double bbox_min_x = 0.0;
	double bbox_min_y = 0.0;
	double bbox_min_z = 0.0;
	double bbox_max_x = 0.0;
	double bbox_max_y = 0.0;
	double bbox_max_z = 0.0;
	bool apply_bounding_box = false;
	std::vector<std::string> input_files;

	// Parse the command line arguments
	for (int i = 1; i < argc; ++i)
	{
		if (strcmp(argv[i], "-o") == 0 && i < argc - 1)
		{
			i++;
			output_filename = argv[i];
		}
		else if (strcmp(argv[i], "--bb") == 0 && i < argc - 7)
		{
			i++;
			bbox_min_x = atof(argv[i]);
			i++;
			bbox_min_y = atof(argv[i]);
			i++;
			bbox_min_z = atof(argv[i]);
			i++;
			bbox_max_x = atof(argv[i]);
			i++;
			bbox_max_y = atof(argv[i]);
			i++;
			bbox_max_z = atof(argv[i]);
			apply_bounding_box = true;
		}
		else
		{
			input_files.push_back(argv[i]);
		}
	}

	//pcl::PointCloud<pcl::PointXYZ> cloud;//comment yqy
	pcl::PointCloud<pcl::PointXYZRGB> cloud;//add yqy
	//std::vector<Voxel> voxels; // for debugging

	for (size_t i = 0; i < input_files.size(); ++i)
	{
		const std::string input_file(input_files.at(i));

		std::ifstream* input = new std::ifstream(input_file, std::ios::in | std::ios::binary);
		if (!input->good())
		{
			std::cerr << "Error: Could not open input file " << input_file << "!" << std::endl;
			exit(1);
		}
		else
		{
			std::cout << "Reading binvox file " << input_file << "." << std::endl;

			if (output_filename.empty())
			{
				output_filename = std::string(input_file).append(".pcd");
			}
		}

		// Read binvox header
		std::string line;
		*input >> line;
		if (line.compare("#binvox") != 0)
		{
			std::cerr << "Error: first line reads [" << line << "] instead of [#binvox]" << std::endl;
			delete input;
			return 0;
		}
		int binvox_version;
		*input >> binvox_version;
		std::cout << "Detected binvox version " << binvox_version << std::endl;

		unsigned int voxel_grid_depth;
		unsigned int voxel_grid_height;
		unsigned int voxel_grid_width;
		double tx;
		double ty;
		double tz;
		double scale;

		voxel_grid_depth = -1;
		int done = 0;
		while (input->good() && !done)
		{
			*input >> line;
			if (line.compare("data") == 0)
			{
				done = 1;
			}
			else if (line.compare("dim") == 0)
			{
				*input >> voxel_grid_depth >> voxel_grid_height >> voxel_grid_width;
			}
			else if (line.compare("translate") == 0)
			{
				*input >> tx >> ty >> tz;
			}
			else if (line.compare("scale") == 0)
			{
				*input >> scale;
			}
			else
			{
				std::cout << "Unsuported keyword found in file [" << line << "], skipping" << std::endl;
				char c;
				// Jump to end of line
				do
				{
					c = input->get();
				} while (input->good() && (c != '\n'));
			}
		}
		if (!done)
		{
			std::cerr << "Error reading header" << std::endl;
			return 0;
		}

		if (voxel_grid_depth == -1)
		{
			std::cout << "Error: missing dimensions in header" << std::endl;
			return 0;
		}

		const int voxel_grid_size = voxel_grid_depth;

		std::cout << "Voxel grid size: " << voxel_grid_size << std::endl;
		std::cout << "Normalization transform: (1) translate ["
			<< tx << ", " << ty << ", " << tz << "], " << std::endl;
		std::cout << "                         (2) scale " << scale << std::endl;

		if (apply_bounding_box)
		{
			std::cout << "Bounding box: ["
				<< bbox_min_x << "," << bbox_min_y << "," << bbox_min_z << " - "
				<< bbox_max_x << "," << bbox_max_y << "," << bbox_max_z << "] \n";
		}

		std::cout << "Read data: ";
		std::cout.flush();

		// Process voxel data
		unsigned char value;
		unsigned char count;
		int index = 0;
		int end_index = 0;
		unsigned num_voxels_read = 0;
		unsigned num_voxels_skipped = 0;

		input->unsetf(std::ios::skipws);
		*input >> value; // read the linefeed char

		const int num_voxels = voxel_grid_width * voxel_grid_height * voxel_grid_depth;


		/*while ((end_index < num_voxels) && input->good())
		{

		}*/
		//add yqy 
		std::vector<char> value2;
		//unsigned  char value2[4.0*num_voxels];
		//while ((end_index < 4.0*num_voxels) && input->good())
		////for (long int i= 0; i < 4.0*num_voxels;++i)	//一定要有input->good()的判断，因为4.0*num_voxels是可能的最大值（非常大的一个数），但实际数目远小于这个值，仅仅使用for速度非常慢出不了结果
		//{
		//	unsigned char value3;
		//	*input >> value3;
		//	value2.push_back(value3);
		//	
		//}
		//不能来两次while ((end_index < 4.0*num_voxels) && input->good())，否则当第二个while开始时数据已经全部被读过一遍没数据可读了，应该放到一个while里进行操作
		//add end


		/*std::ifstream infile;
		infile.open("texture.txt");*/
		//二进制
		std::ifstream infile("texture.txt", std::ios::binary);
		if (!infile)
		{
			std::cerr << "open error!" << std::endl;
			abort();
		}

		
		//std::vector<char> str2;
		//char strchar;
		//std::string str;
		//while(infile.read((char*)&strchar, sizeof(strchar)))
		//{
		//	//infile.read((char*)&strchar, sizeof(strchar));
		//	str2.push_back(strchar);
		//}



		//std::vector<int> str2;
		//char strchar;
		//std::string str;
		//while (infile.read((char*)&strchar, sizeof(strchar)))
		//{
		//	//infile.read((char*)&strchar, sizeof(strchar));
		//	str2.push_back(strchar);
		//}

		float str;
		std::vector<float> str2;
		int testyq = 0;
		while (infile >> str)
		{
			str2.push_back(str);
			//str2.push_back(testyq);
			//testyq += 1;
			//std::cout << str << std::endl;
		}
		infile.close();

		unsigned int countallpoint = 0;
		unsigned int real_num_voxels = 0;//add yqy  从pcd2voxel中的countallpoint而来
		//while ((end_index < 4.0*num_voxels) && input->good())//add yqy
		while ((end_index < num_voxels) && input->good())//comement yqy//good()函数用来判断当前流状态是否健康，当遇到EOF、输入类型不匹配的时候放回false
		//while ((end_index < real_num_voxels) && input->good())//add yqy
		{
			*input >> value >> count;
			/*countallpoint += 1;*/


			//unsigned char value3;
			//*input >> value3;
			//value2.push_back(value3);

			if (input->good())
			{
				end_index = index + count;

				if (end_index > num_voxels)
				{
					return 0;
				}

				for (int i = index; i < end_index; ++i)//comment yqy
				//for (int i = index; (i < num_voxels)&&(i < end_index); ++i)//add yqy
				{
					// Output progress dots  代表进程状态的点
					if (i % (num_voxels / 20) == 0)
					{
						std::cout << ".";
						std::cout.flush();
					}

					////add yqy
					//float iy = 0;
					//float iz = 0;
					//float ix = 0;
					//if (end_index < num_voxels)//add yqy
					//{
					//	// Get the voxel indices
					//	iy = static_cast<float>(i % voxel_grid_width);
					//	iz = static_cast<float>((i / voxel_grid_width) % voxel_grid_height);
					//	ix = static_cast<float>(i / (voxel_grid_width * voxel_grid_height));
					//}
					////yqy end

					//comment yqy
					// get the voxel indices
					const float iy = static_cast<float>(i % voxel_grid_width);
					const float iz = static_cast<float>((i / voxel_grid_width) % voxel_grid_height);
					const float ix = static_cast<float>(i / (voxel_grid_width * voxel_grid_height));
					//yqy end

					////add yqy  不对，使用了游程编码，以下这种表示只有在一个一个存的时候才对
					//const float ir = value2[index+ num_voxels];
					//const float ig = value2[index + 2*num_voxels];
					//const float ib = value2[index +3* num_voxels];
					////add end

					// Convert voxel indices from integer to values between 0.0 to 1.0
					// The 0.5 aligns the point with the center of the voxel.
					const float x = (ix + 0.5) / static_cast<float>(voxel_grid_size);
					const float y = (iy + 0.5) / static_cast<float>(voxel_grid_size);
					const float z = (iz + 0.5) / static_cast<float>(voxel_grid_size);

					const double px = scale * static_cast<double>(x) + tx;
					const double py = scale * static_cast<double>(y) + ty;
					const double pz = scale * static_cast<double>(z) + tz;

					//comment yqy
				/*	pcl::PointXYZ point(static_cast<float>(px),
						static_cast<float>(py),
						static_cast<float>(pz));*/
					//comment end

					////add yqy
					//pcl::PointXYZRGB point();//这样会报错表达式必须包含类类型
					/*pcl::PointXYZRGB point(static_cast<float>(px),
						static_cast<float>(py),
						static_cast<float>(pz));*///其实并没有准确赋值，因为PointXYZRGB初始化是给rgb初始化
				/*	point.r = ir;
					point.g = ig;
					point.b = ib;*/
					////add end

					//add yqy
					//if (value == 1)
					//{
					//	/*const float ir = str2[countallpoint + 0];
					//	const float ig = str2[countallpoint + 1];
					//	const float ib = str2[countallpoint + 2];*/
					//	int ir = str2[countallpoint + 0];
					//	int ig = str2[countallpoint + 1];
					//	int ib = str2[countallpoint + 2];


					//	pcl::PointXYZRGB point(ir,
					//		ig,
					//		ib);


					//	countallpoint += 1;
					//	point.x = px;
					//	point.y = py;
					//	point.z = pz;
					//	//point.r = ir;
					//	//point.g = ig;
					//	//point.b = ib;//赋值过来变成了ascii码对应的字母，而不是真正的rgb值  这说明rgb的赋值采用的是ascii码，要修改赋值方式
					//}
					
					//add end

					if (!apply_bounding_box
						|| (px <= bbox_max_x && px >= bbox_min_x
							&& py <= bbox_max_y && py >= bbox_min_y
							&& pz <= bbox_max_z && pz >= bbox_min_z))
					{
						if (value == 1)//1是真正的点云，0代表没有物体所在的位置
						{


						
							int ir = str2[countallpoint + 0];
							int ig = str2[countallpoint + 1];
							int ib = str2[countallpoint + 2];

							//int ir = str2[countallpoint + 0];
							//int ig = 0;
							//int ib = 0;



							//const float ir = str2[countallpoint + 0];
							////ir=百位上的数字*100
							//const float ig = str2[countallpoint + 1];
							//const float ib = str2[countallpoint + 2];
							//pcl::PointXYZRGB point;
							pcl::PointXYZRGB point = pcl::PointXYZRGB(static_cast<float>(ir),
								static_cast<float>(ig),
								static_cast<float>(ib));
							//pcl::PointXYZRGB point(255,0,0);
							//并没有当做ascii码赋值的问题，输入pcl::PointXYZRGB point(255,0,0);可以得到红色

							countallpoint += 3;//加3啊不是加1！！！！！！！！！太坑啦
							point.x = px;
							point.y = py;
							point.z = pz;



							//const float ir = str2[countallpoint + 0];
							//const float ig = str2[countallpoint + 1];
							//const float ib = str2[countallpoint + 2];
							//countallpoint += 1;
							//point.x = px;
							//point.y = py;
							//point.z = pz;
							/*point.r = static_cast<float>(57);
							point.g = 0;
							point.b = 255;*/


							/*uint32_t rgb = (static_cast<uint32_t>(ir) << 16 |
								static_cast<uint32_t>(ig) << 8 | static_cast<uint32_t>(ib));
							point.rgb = *reinterpret_cast<float*>(&rgb);*/

							//经过验证，以下这种方式可以正常赋值，可以得到全红的
							//uint8_t r = 255, g = 0, b = 0;
							//uint8_t r = str2[countallpoint + 0], g = str2[countallpoint + 1], b = str2[countallpoint + 2];    // Example: Red color
							//uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
							//point.rgb = *reinterpret_cast<float*>(&rgb);

							cloud.points.push_back(point);

							


							// For debugging: Save the voxel indices
							//voxels.push_back(Voxel(static_cast<unsigned int>(ix),
							//                       static_cast<unsigned int>(iy),
							//                       static_cast<unsigned int>(iz)));
						}
					}
					else
					{
						num_voxels_skipped++;
					}
				}

				if (value)
				{
					num_voxels_read += count;
				}

				index = end_index;
			}
		}
		/*std::cout << "countallpoint " << countallpoint << std::endl;
		std::cout << "num_voxels " << num_voxels << std::endl;*/

		std::cout << std::endl << std::endl;

		input->close();
		std::cout << "Read " << num_voxels_read << " voxels";
		if (num_voxels_skipped > 0)
		{
			std::cout << ", skipped " << num_voxels_skipped << " (outside bounding box)";
		}

		std::cout << "\n" << std::endl;
	} // end processing each binvox file

	  // For debugging: Write voxel indices to a file
	  //writeVoxelsToFile(voxels, "voxels_from_binvox.txt");

	std::cout << "PointCloud has " << cloud.points.size() << " points";
	//comment yqy
	/*pcl::PointXYZ min_point;
	pcl::PointXYZ max_point;*/
	//comment end
	//add yqy
	pcl::PointXYZRGB min_point;
	pcl::PointXYZRGB max_point;
	//add end
	pcl::getMinMax3D(cloud, min_point, max_point);
	std::cout << ", with extents: "
		<< "(" << min_point.x << ", " << min_point.y << ", " << min_point.z << ") to "
		<< "(" << max_point.x << ", " << max_point.y << ", " << max_point.z << ")" << std::endl;

	pcl::PCDWriter writer;
	writer.writeBinaryCompressed(output_filename, cloud);

	std::cout << "done" << std::endl << std::endl;
	return 0;
}
#endif












