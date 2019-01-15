#ifndef GNCTK_FUSION_QUAD_TREE_H_
#define GNCTK_FUSION_QUAD_TREE_H_

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

fusion_quad_tree.h

quad tree based camera lidar fusion object
----------------------------------------------

These source files contain two objects;

QuadTreeNode - a recurrsive quad tree object used to store
and manage the saliency quad tree for lidar surface
reconstruction.

FusionQuadTree - a concrete sub-class of the Fusion class
which implements the quad tree heat map method of lidar
camera fusion.

-------------------------------------------------------------*/
#include <stdio.h>
#include "cv.h"
#include <pcl_ros/point_cloud.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include <GLFW/glfw3.h>
#include "opencv2/highgui/highgui.hpp"
#include "visualization_msgs/Marker.h"
#include "mesh.h"
#include "fusion.h"

namespace gncTK {
	class FusionQuadTree;
	class QuadTreeNode;
};

class gncTK::QuadTreeNode
{
public:

	QuadTreeNode(Eigen::Vector2f _topLeft, Eigen::Vector2f _bottomRight)
	{
		topLeft = _topLeft;
		bottomRight = _bottomRight;
		pointCount = 0;
		meanPoint << 0,0,0;
		meanIntensity = 0.0f;
		pointCount = 0;
		depth = 0;
		isLeaf = true;
		meshVertexIndex = -1;

		sumD = sumD2 = 0.0;
		m2 = m3 = m4 = 0.0;
	};
	~QuadTreeNode()
	{
		if (!isLeaf)
			delete tlChild, trChild, blChild, brChild;
	};

	void splitToCount(float targetCount, std::vector<cv::Mat> *heatMaps);
	void split();
	bool mergeLeaves(bool reccursive = false);
	QuadTreeNode* findNode(Eigen::Vector2f pos);
	int count();
	int countNonZero();

	void filterLeaves(int minPointCount, int minNeighbourCount);

	void addVertices(gncTK::Mesh *mesh);

	// Method to populate the neighbour links of all leaves in the tree recursively
	void generateNeighbourLinks();

	// Helper functions for neighbour link generation
	std::vector<QuadTreeNode*> getRightEdgeLeaves();
	std::vector<QuadTreeNode*> getLeftEdgeLeaves();
	std::vector<QuadTreeNode*> getTopEdgeLeaves();
	std::vector<QuadTreeNode*> getBottomEdgeLeaves();

	// Method to triangulate this quad tree into the given mesh
	void generateTriangles(gncTK::Mesh *mesh);

	/// Method to add leaf statistics to the given images
	void addStats(int rows, int cols,
			      cv::Mat *N,
				  cv::Mat *meanImg,
				  cv::Mat *stdDevImg,
				  cv::Mat *skewImg,
				  cv::Mat *kurtImg);

	QuadTreeNode *tlChild, *trChild, *blChild, *brChild;

	// Neighbour links used by meshing algorithm
	std::vector<QuadTreeNode*> topNs, leftNs, bottomNs, rightNs;

	bool isLeaf;
	int depth;

	Eigen::Vector2f topLeft, bottomRight;
	double heatMapValue;
	double meanHeatMapValue;

	Eigen::Vector3f meanPoint;
	double meanIntensity;
	int pointCount;

	// stats accumulators

	double sumD, sumD2;
	double m2, m3, m4;

	int meshVertexIndex;

	std::string frameId;
};

// -------------------------------------------------------------------------------------

class gncTK::FusionQuadTree : public gncTK::Fusion
{
public:

	FusionQuadTree();
	~FusionQuadTree();

	gncTK::Mesh generateMesh();

	/// overloaded point cloud input method for this fusion methodology
	void setInputCloud(pcl::PointCloud<pcl::PointXYZI> cloud);

	/// Method to create the quad tree from a mono float heat map image
	void setQuadTreeHeatMap(cv::Mat heatMap, int leafCount = 10000, double gamma = 1.0);

	/// Method to create a set of images covering the quad tree area showing stats for each leaf
	std::vector<cv::Mat> exportLeafStats();

	//cv::Mat generateDepthImage(int resolutionFactor = 1, int border = 0, Eigen::Vector3f gravity = Eigen::Vector3f::Zero());

	void setIncidentAngleThreshold(float angle);

private:

	bool isValid(pcl::PointXYZI point)
	{
		if (point.x == 0 && point.y == 0 && point.z == 0)
			return false;

		return (!std::isnan(point.x) &&
				!std::isnan(point.y) &&
				!std::isnan(point.z));
	}

	/// Private method used to build the image pyramids (mid-maps) from power of two input image
	std::vector<cv::Mat> buildImagePyramid(cv::Mat input);

	void setupOffscreenGLBuffer(int width, int height);

	std::vector<float> calculateGravityAngles(gncTK::Mesh mesh, Eigen::Vector3f gravity);

	std::vector<float> estimateVertexHeights(gncTK::Mesh mesh, float gridSize, Eigen::Vector3f gravity);

	// feature size threshold for mesh simplification
	float featureSize;
	float incidentAngleThreshold;

	/// Pointer to GLFW window used for OpenGL rendering
	GLFWwindow* window;
	bool glContextSetup;
	GLuint ssFbo, ssColorBuf, ssDepthBuf;

	std::string frameId;
	Eigen::Vector3f sensorOrigin;

	bool quadTreeSet;
	gncTK::QuadTreeNode *treeRoot;
};

#endif /* GNCTK_FUSION_QUAD_TREE_H_ */
