/*
 * module_manager.cpp
 *
 *  Created on: 25 Nov 2018
 *      Author: user
 */

#include <vector>
#include <string>
#include <ros/ros.h>
#include <tf/tf.h>
#include <urdf/model.h>
#include <stdio.h>

class ModuleConnector
{
	tf::Transform pose;
};

class ModuleType
{
public:
	ModuleType(std::string typeName, std::string URDFString);

	bool fixedJointExists(urdf::Model *model, std::string linkName);

	tf::Transform getFixedJointTF(boost::shared_ptr< const urdf::Joint > joint);

	std::string name;

	bool loadedOkay;

	urdf::Model urdf;

	// list of connectors for the module
	std::vector<tf::Transform> connectors;

	// collision model
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

			tf::Transform connectorTF = getFixedJointTF(connectorJoint);
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

tf::Transform ModuleType::getFixedJointTF(boost::shared_ptr< const urdf::Joint > joint)
{
	tf::Transform linkTF;

	tf::Vector3 origin(joint->parent_to_joint_origin_transform.position.x,
					   joint->parent_to_joint_origin_transform.position.y,
					   joint->parent_to_joint_origin_transform.position.z);
	tf::Quaternion rotation(joint->parent_to_joint_origin_transform.rotation.x,
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
	}

	ModuleType *type;
	std::string id;

	tf::Transform poseRelativeToRoot;

	std::vector<ModuleConnector> connectors;
};

class ModuleManager
{
public:

	ModuleManager() : rootSet(false) {};

	ModuleInstance* getRootModule();

	bool validateModuleStructure();

	void publishTFs();

	void updateMoveitCollisionScene();

	bool loadConfiguration(std::string name);

	void reconnectSim(/* representation */);

	void reconnectReal(/* representation */);

	bool ensureModuleTypeIsLoaded(std::string type);

	ModuleType *getModuleType(std::string type);

	std::vector<ModuleInstance> modules;

	ModuleInstance *rootModule;
	bool rootSet;

	std::map<std::string, ModuleType> loadedModuleTypes;
};

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

		mainRate.sleep();
	}

	return 0;
}




















