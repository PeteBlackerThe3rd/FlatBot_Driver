/*
 * module_manager.cpp
 *
 *  Created on: 25 Nov 2018
 *      Author: user
 */

#include <vector>
#include <string>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
//#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf/tf.h>
#include <urdf/model.h>
#include <stdio.h>

class ModuleConnector
{
	tf2::Transform pose;
};

class ModuleType
{
public:
	ModuleType(std::string typeName, std::string URDFString);

	bool fixedJointExists(urdf::Model *model, std::string linkName);

	tf2::Transform getFixedJointTF(boost::shared_ptr< const urdf::Joint > joint);

	std::string name;

	bool loadedOkay;

	urdf::Model urdf;

	// list of connectors for the module
	std::vector<tf2::Transform> connectors;

	// collision model filename
	std::string collisionFileName;

	// visual mode filename
	std::string visualFileName;
};

ModuleType::ModuleType(std::string typeName, std::string URDFString)
{
	name = typeName;

	if (!urdf.initString(URDFString.c_str()))
	{
		ROS_ERROR("Failed to parse urdf string for module type [%s]", typeName.c_str());
		loadedOkay = false;
	}
	else
	{
		printf("loaded urdf string okay!\n");

		// each URDF for a module must have a base link and a set of connector links with fixed joints to the base

		boost::shared_ptr<const urdf::Link> baseLink = urdf.getRoot();

		// get the visual model of this module
		if (baseLink->visual->geometry->type == urdf::Geometry::MESH)
		{
			boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(baseLink->visual->geometry);

			visualFileName = mesh->filename;
			//ROS_INFO("Read module visual filename of [%s]", visualFileName.c_str());
		}
		else
		{
			ROS_ERROR("Error no visual mesh file specified for module!");
			visualFileName = "NotSet";
		}

		// loop through each link checking it has a fixed joint to the base link
		// node links may be out of order!
		std::vector<boost::shared_ptr<urdf::Link> > links;
		urdf.getLinks(links);

		for (int l=0; l<links.size(); ++l)
		{
			if (links[l] == baseLink)
				continue;

			// the link names must be "connector_X" where x is between 0-9
			if (links[l]->name.compare(0, 10, "connector_") != 0)
			{
				ROS_ERROR("Error processing \"%s\" module URDF: connector link name doesn't start with \"connector_\"\n", name.c_str());
				return;
			}

			if (links[l]->name.length() != 11)
			{
				ROS_ERROR("Error processing \"%s\" module URDF: connector link name incorrect length\n", name.c_str());
				return;
			}

			if (links[l]->name.c_str()[10] < '0' || links[l]->name.c_str()[10] > '9')
			{
				ROS_ERROR("Error processing \"%s\" module URDF: connector link name incorrect length\n", name.c_str());
				return;
			}

			int linkNum = atoi(links[l]->name.substr(10,1).c_str());

			boost::shared_ptr< const urdf::Joint > connectorJoint = urdf.getJoint(links[l]->name + "_joint");

			if (!connectorJoint || connectorJoint->type != urdf::Joint::FIXED)
			{
				ROS_ERROR("Error processing \"%s\" module URDF: no fixed joint to base link\n", name.c_str());
				return;
			}

			//printf("Successfully found connector [%d] of module \"%s\"\n", linkNum, name.c_str());

			tf2::Transform connectorTF = getFixedJointTF(connectorJoint);
			printf("made TF okay.\n"); fflush(stdout);
			if (connectors.size() < linkNum+1)
				connectors.resize(linkNum+1);
			connectors[linkNum] = connectorTF;
		}

		if (connectors.size() == links.size()-1)
			ROS_INFO("Correctly loaded module with [%d] connectors", (int)connectors.size());

		// TODO load collision model mesh!!

		loadedOkay = true;
	}
}

tf2::Transform ModuleType::getFixedJointTF(boost::shared_ptr< const urdf::Joint > joint)
{
	tf2::Transform linkTF;

	tf2::Vector3 origin(joint->parent_to_joint_origin_transform.position.x,
					    joint->parent_to_joint_origin_transform.position.y,
					    joint->parent_to_joint_origin_transform.position.z);
	tf2::Quaternion rotation(joint->parent_to_joint_origin_transform.rotation.x,
							 joint->parent_to_joint_origin_transform.rotation.y,
							 joint->parent_to_joint_origin_transform.rotation.z,
							 joint->parent_to_joint_origin_transform.rotation.w);
	linkTF.setOrigin(origin);
	linkTF.setRotation(rotation);
	return linkTF;
}

class ModuleInstance
{
public:

	ModuleInstance(ModuleType *modType, std::string id)
	{
		type = modType;
		this->id = id;
		poseRelativeToRootKnown = false;
	}

	ModuleType *type;
	std::string id;

	/// the pre-calculated pose of this module relative to the rood modules it is static relative to
	tf2::Transform poseRelativeToRoot;
	bool poseRelativeToRootKnown;

	/// rootFrameId will be either the bus_frame or a robots EE frame.
	std::string rootFrameId;

	std::vector<ModuleConnector> connectors;
};

class ModuleManager
{
public:

	ModuleManager() : rootSet(false), modulePosesUptoDate(false)
	{
		ros::NodeHandle n;
		moduleMarkerPub = n.advertise<visualization_msgs::MarkerArray>("module", 10);
	};

	ModuleInstance* getRootModule();

	bool validateModuleStructure();

	void publishTFs();

	void publishModuleMarkers();

	void updateMoveitCollisionScene();

	bool loadConfiguration(std::string name);

	void reconnectSim(/* representation */);

	void reconnectReal(/* representation */);

	bool ensureModuleTypeIsLoaded(std::string type);

	ModuleType *getModuleType(std::string type);

private:

	/// Helper function to form the module frame id strings
	std::string makeModuleFrameId(std::string moduleId);

	/// Helper function to refresh the caches poses of all the modules
	void updateModulePoses();

	/// Helper function to convert a tf2_transform object to a geometry_msgs::Transform message
	geometry_msgs::Transform transformToTransformMsg(tf2::Transform transform);

	std::vector<ModuleInstance> modules;
	bool modulePosesUptoDate;

	ModuleInstance *rootModule;
	bool rootSet;

	std::map<std::string, ModuleType> loadedModuleTypes;

	ros::Publisher moduleMarkerPub;
};

void ModuleManager::publishTFs()
{
	static tf2_ros::TransformBroadcaster br;

	ros::Time timePoint = ros::Time::now();

	if(!modulePosesUptoDate)
		updateModulePoses();

	for (int m=0; m<modules.size(); ++m)
	{
		// if this is not the root module then publish it's TF relative to it's root node.
		if (modules[m].id != rootModule->id)
		{
			geometry_msgs::TransformStamped transformStamped;

			transformStamped.header.stamp = timePoint;
			transformStamped.header.frame_id = modules[m].rootFrameId;
			transformStamped.child_frame_id = makeModuleFrameId(modules[m].id);
			transformStamped.transform = transformToTransformMsg(modules[m].poseRelativeToRoot);

			br.sendTransform(transformStamped);
		}

		// add the connector frames
		ModuleType *type = modules[m].type;
		for (int c=0; c<type->connectors.size(); ++c)
		{
			geometry_msgs::TransformStamped transformStamped;

			transformStamped.header.stamp = ros::Time::now();
			transformStamped.header.frame_id = makeModuleFrameId(modules[m].id);
			transformStamped.child_frame_id = makeModuleFrameId(modules[m].id) + "_connector_" + std::to_string(c);
			transformStamped.transform = transformToTransformMsg(type->connectors[c]);

			br.sendTransform(transformStamped);
		}
	}
}

std::string ModuleManager::makeModuleFrameId(std::string moduleId)
{
	return "module_" + moduleId + "_tf";
}

void ModuleManager::updateModulePoses()
{
	std::string busRootFrameId = makeModuleFrameId(rootModule->id);
	std::string manipulatorEEFrameId = "ee_frame";

	// reset known module positions execpt for root
	for (int m=0; m<modules.size(); ++m)
	{
		modules[m].poseRelativeToRootKnown = (modules[m].id == rootModule->id);
	}

	// repeatedly loop through the modules adding the known positions of new modules
	// if their direct parents are known.
	// stop looping if no more modules are known by a single iteration
	//bool modulesAdded = true;
	//while(modulesAdded)
	//{
		for (int m=0; m<modules.size(); ++m)
		{
			tf2::Transform transform;
			transform.setOrigin( tf2::Vector3(m*1.0, m*0.5, m*0.2) );
			tf2::Quaternion q;
			q.setRPY(0, 0, m);
			transform.setRotation(q);

			// define the pose of the module
			modules[m].poseRelativeToRoot = transform;

			// define what this pose is relative to
			modules[m].rootFrameId = busRootFrameId;
		}
	//}

	modulePosesUptoDate = true;
}

geometry_msgs::Transform ModuleManager::transformToTransformMsg(tf2::Transform transform)
{
	geometry_msgs::Transform msg;

	msg.translation.x = transform.getOrigin()[0];
	msg.translation.y = transform.getOrigin()[1];
	msg.translation.z = transform.getOrigin()[2];

	msg.rotation.x = transform.getRotation().x();
	msg.rotation.y = transform.getRotation().y();
	msg.rotation.z = transform.getRotation().z();
	msg.rotation.w = transform.getRotation().w();

	return msg;
}

void ModuleManager::publishModuleMarkers()
{
	visualization_msgs::MarkerArray ma;

	// add markers for each module
	for (int m=0; m<modules.size(); ++m)
	{
		visualization_msgs::Marker marker;

		marker.header.stamp = ros::Time::now();
		marker.header.frame_id = makeModuleFrameId(modules[m].id);
		marker.ns = "module_markers";
		marker.id = m;
		marker.type = visualization_msgs::Marker::MESH_RESOURCE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 1;
		marker.scale.y = 1;
		marker.scale.z = 1;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		//only if using a MESH_RESOURCE marker type:
		marker.mesh_resource = modules[m].type->visualFileName;

		ma.markers.push_back(marker);
	}

	moduleMarkerPub.publish(ma);
}

bool ModuleManager::ensureModuleTypeIsLoaded(std::string type)
{
	// if this module is already loaded then it's okay
	std::map<std::string, ModuleType>::iterator modType = loadedModuleTypes.find(type);

	if (modType != loadedModuleTypes.end())
		return true;

	// if not try and load the module
	ros::NodeHandle nh("~");
	std::string moduleParamName = "module_description_" + type;
	std::string moduleURDFString;
	if(!ros::param::get(moduleParamName, moduleURDFString))
	{
		ROS_ERROR("Error loading module type \"%s\" URDF parameter \"%s\" not present.", type.c_str(), moduleParamName.c_str());
		return false;
	}

	ModuleType newModuleType(type, moduleURDFString);

	if (newModuleType.loadedOkay)
		loadedModuleTypes.emplace(type, newModuleType);

	return true;
}

ModuleType *ModuleManager::getModuleType(std::string type)
{
	// find this module type in map
	std::map<std::string, ModuleType>::iterator modType = loadedModuleTypes.find(type);

	if (modType != loadedModuleTypes.end())
		return &modType->second;
	else
		return NULL;
}

bool ModuleManager::loadConfiguration(std::string name)
{
	// YAML config data structure
	XmlRpc::XmlRpcValue moduleYaml;

	std::string configParamName = "/" + name;

	if(!ros::param::get(configParamName.c_str(), moduleYaml))
	{
		ROS_ERROR("Failed to read module config from param server");
		exit(1);
	}

	if (moduleYaml.getType() == XmlRpc::XmlRpcValue::TypeArray)
	{
		printf("Modules is an array, that's good I guess.\n");

		printf("size of modules is [%d]\n", moduleYaml.size());
	}

	for (int i=0; i<moduleYaml.size(); ++i)
	{
		if (moduleYaml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
			printf("module [%d] is a struct, good I guess!\n", i);

		std::string id = moduleYaml[i]["id"];
		std::string type = moduleYaml[i]["type"];
		bool isRoot = moduleYaml[i]["root"];

		if (id != "" && type != "")
		{
			printf("Found a module definition okay.\nid : %s\ntype : %s\n", id.c_str(), type.c_str());
			if (isRoot)
				printf("Root module\n");
			else
				printf("Not root module\n");

			if (ensureModuleTypeIsLoaded(type))
			{
				printf("module type is okay.\n");

				ModuleInstance newModule(getModuleType(type), id);
				modules.push_back(newModule);

				// if this is the root module then add a link to.
				if (isRoot)
					rootModule = &modules[modules.size()-1];
			}
			else
				printf("failed to load module type!\n");
		}

	}
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "module_manager");
	ros::start();

	ros::NodeHandle nh("~");

	ModuleManager modman;

	modman.loadConfiguration("modules");

	ros::Rate mainRate(10.0);

	while(ros::ok())
	{
		modman.publishTFs();
		modman.publishModuleMarkers();

		mainRate.sleep();
	}

	return 0;
}




















