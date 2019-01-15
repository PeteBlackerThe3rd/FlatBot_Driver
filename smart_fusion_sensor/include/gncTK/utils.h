#ifndef GNCTK_UTILS_H_
#define GNCTK_UTILS_H_

/*-----------------------------------------------------------\\
||                                                           ||
||                 LIDAR fusion GNC project                  ||
||               ----------------------------                ||
||                                                           ||
||    Surrey Space Centre - STAR lab                         ||
||    (c) Surrey University 2017                             ||
||    Pete dot Blacker at Gmail dot com                      ||
||                                                           ||
\\-----------------------------------------------------------//

utils.h

general helper functions and data structures
---------------------------------------------------



-------------------------------------------------------------*/

#include "cv.h"
#include "opencv2/highgui/highgui.hpp"
#include <string>
#include <pcl_ros/point_cloud.h>

namespace gncTK
{
	class Utils;
}

class gncTK::Utils
{
public:

	/// Method to generate a point cloud which represents the edges of an orthogonal cuboid, usefull for simple RVIS visualisations
	static pcl::PointCloud<pcl::PointXYZRGB> pointCloudBox(float sX,
														   float sY,
														   float sZ,
														   std::string frameId,
														   float spacing=0.1);

	/// Methods for working with HD matricies
	static Eigen::Matrix4d normalizeDHMatrix(Eigen::Matrix4d matrix);
	static void analyseDHMatrix(Eigen::Matrix4d matrix);

	// Methods to load and save CSV files easily
	static int readCSVFile(FILE *file, std::vector<std::vector<std::string> > *content,
						   char separator=',',
						   int skip=0);
	static int readCSVFile(std::string fileName, std::vector<std::vector<std::string> > *content,
						   char separator=',',
						   int skip=0);
	static void writeCSVFile(FILE *file,
							 std::vector<std::vector<std::string> > *content);
	static void writeCSVFile(std::string fileName,
							 std::vector<std::vector<std::string> > *content);
	static void writeCSVFile(FILE *file,
							 std::vector<std::vector<float> > content);
	static void writeCSVFile(std::string fileName,
							 std::vector<std::vector<float> > content);

	// Methods to process file name strings
	static std::string fileNameWOPath(std::string fileName);
	static std::string fileNameWOExtension(std::string fileName);

	/// Method to return the HSV rainbow color for the position given by color, values are clamped between 0 and 1.
	static cv::Vec3b rainbow(float color);
	static cv::Mat rainbow(cv::Mat input);

	/// Method to convert a single channel of a floating point CV image as an 8 bit image with a scale bar
	static cv::Mat floatImageTo8Bit(cv::Mat floatImage, int channel=-1, bool useRainbow = true);

	/// Method to save a floating point openCV image to a file using custom format
	static void imwriteFloat(FILE *file, cv::Mat image);
	static void imwriteFloat(std::string fileName, cv::Mat image);

	/// Method to load a floating point openCV image to a file using custom format
	static cv::Mat imreadFloat(std::string fileName);
};

#endif /* GNCTK_UTILS_H_ */
