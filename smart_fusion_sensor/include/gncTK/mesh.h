#ifndef GNCTK_MESH_H_
#define GNCTK_MESH_H_

/*-----------------------------------------------------------\\
||                                                           ||
||                 GNC Toolkit Library                       ||
||               -----------------------                     ||
||                                                           ||
||    Surrey Space Centre - STAR lab                         ||
||    (c) Surrey University 2017                             ||
||    Pete dot Blacker at Gmail dot com                      ||
||                                                           ||
\\-----------------------------------------------------------//

mesh.h

triangular mesh storage object
----------------------------

This object stores various different forms of triangulated
mesh. It can include single or multiple texture information
as well as per vertex color and normal information

Helper functions are provided to load and save this data
in wavefront OBJ and stanford PLY formats.

Functions are also provided to convert the data to PCL
Point cloud objects or ROS mesh marker messages

An efficient geometric deviation calculation is also provided
to calculate the geometric deviations between this mesh and
another.

-------------------------------------------------------------*/
#include <stdio.h>
#include "cv.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "opencv2/highgui/highgui.hpp"
#include "visualization_msgs/Marker.h"
#include "tf/tf.h"
//#include "tf2_eigen/tf2_eigen.h"
#include "tf/transform_datatypes.h"

namespace gncTK
{
	class Mesh;
};

class gncTK::Mesh
{
public:

	Mesh()
	{
		KDTreeCached = false;
		edgesCalculated = false;
	}

	class Triangle
	{
	public:
		Triangle(int _v1, int _v2, int _v3)
		{
			v1 = _v1;
			v2 = _v2;
			v3 = _v3;
			texId = 0;
		}
		Triangle(int _v1, int _v2, int _v3, int _texId)
		{
			v1 = _v1;
			v2 = _v2;
			v3 = _v3;
			texId = _texId;
		}
		Triangle(int _v1, int _v2, int _v3,
							  int _t1, int _t2, int _t3,
							  int _texId)
		{
			v1 = _v1;
			v2 = _v2;
			v3 = _v3;
			t1 = _t1;
			t2 = _t2;
			t3 = _t3;
			texId = _texId;
		}
		Triangle(int _v1, int _v2, int _v3,
							  int _t1, int _t2, int _t3,
							  int _n1, int _n2, int _n3,
							  int _texId)
		{
			v1 = _v1;
			v2 = _v2;
			v3 = _v3;
			t1 = _t1;
			t2 = _t2;
			t3 = _t3;
			n1 = _n1;
			n2 = _n2;
			n3 = _n3;
			texId = _texId;
		}
		void operator=(const gncTK::Mesh::Triangle& b)
		{
			v1 = b.v1;
			v2 = b.v2;
			v3 = b.v3;
			t1 = b.t1;
			t2 = b.t2;
			t3 = b.t3;
			n1 = b.n1;
			n2 = b.n2;
			n3 = b.n3;
			texId = b.texId;
		}

		int v1,v2,v3;
		int t1,t2,t3;
		int n1,n2,n3;
		int texId;
	};

	class Texture
	{
	public:
		cv::Mat texture;
		std::string fileName;
		std::string label;
		unsigned int glId;
	};

	std::vector<Eigen::Vector3f> vertices;
	std::vector<std::vector<int> > vertexTriangleLinks;
	std::vector<bool> vertexEdges;
	std::vector<Eigen::Vector3f> vertexNormals;
	std::vector<cv::Vec3b> vertexColors;
	std::vector<float> vertexIntensities;
	std::vector<Eigen::Vector2f> texCoords;

	// mesh edge storage, a vector for each vertex in the mesh with a list of
	// vertices of connected edges. Lowest numbered vertex is stored first to avoid duplicates
	std::vector<std::vector<int> > edgeArcs;

	std::vector<Triangle> triangles;
	std::vector<cv::Vec3b> triangleColors;
	std::vector<Texture> textures;

	std::string frameId;
	Eigen::Vector3f sensorOrigin;

	// static mesh factory methods
	static Mesh loadOBJ(std::string fileName);
	static Mesh loadPLY(std::string fileName);

	// methods to save this mesh in various formats
	bool saveOBJ(std::string baseName);
	bool savePLY(std::string fileName);

	// static factory methods to create mesh objects from point cloud objects
	static Mesh fromPCL(pcl::PointCloud<pcl::PointXYZ> pointCloud);
	static Mesh fromPCL(pcl::PointCloud<pcl::PointXYZI> pointCloud);
	static Mesh fromPCL(pcl::PointCloud<pcl::PointXYZRGB> pointCloud);

	// methods to generate a PCL point cloud of this mesh
	pcl::PointCloud<pcl::PointXYZ> toPointCloud();
	pcl::PointCloud<pcl::PointXYZRGB> toPointCloudColor();

	// method to generate a ROS marker message of the mesh
	visualization_msgs::Marker toMarkerMsg(cv::Vec3b tint = cv::Vec3b(0,0,0));

	// method to create a single texture entry for this mesh and to set it to the given openCV mat
	void setSingleTexture(cv::Mat image);

	// method to get the bounding box of this mesh
	Eigen::Matrix<float, 3, 2> getExtents();

	static Eigen::Matrix<float, 3, 2> vertexExtents(std::vector<Eigen::Vector3f> vertices);

	// method to re-calculate/create the vertex normals for this mesh
	void calculateNormals();

	// cached KD tree for fast nearest neighbour lookup
	void setUnusedVerticesToNAN();
	bool KDTreeCached;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;

	/// method to calculate the vertex to triangle links
	void calculateVertexToTriangleLinks();

	/// calculate edge arcs and vertices
	void calculateEdges();

	/// Method which returns true if the given arc is an edge and false otherwise
	/*
	 * The edge arc list must have already been populated using 'calculateEdges()'
	 */
	bool isEdge(int v1, int v2);

private:

	// OBJ specific data structures and methods
	int objLookupMtlName(std::string label);
	void objLoadMaterialLibrary(std::string mtlLibName);

	/// helper function used by calculateEdges() function
	void processEdge(int v1, int v2);
	bool edgesCalculated;
};

#endif /* GNCTK_MESH_H_ */
