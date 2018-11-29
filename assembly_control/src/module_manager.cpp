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

class ModuleConnector
{
	tf::Transform pose;
};

class ModuleType
{
public:
	ModuleType(std::string typeName, std::string URDFString) : name(typeName)
	{
		if (!urdf.initString(URDFString.c_str()))
		{
			ROS_ERROR("Failed to parse urdf string for module type [%s]", typeName.c_str());
			loadedOkay = false;
		}
		else
		{
			printf("loaded urdf string okay!\n");
			loadedOkay = true;
		}
	}

	std::string name;

	bool loadedOkay;

	urdf::Model urdf;

	// collision model
};

class ModuleInstance
{
public:

	tf::Transform poseRelativeToRoot;

	std::vector<ModuleConnector> connectors;
};

class ModuleManager
{
public:

	ModuleManager() {};

	ModuleInstance* getRootModule();

	bool validateModuleStructure();

	void publishTransforms();

	void updateMoveitCollisionScene();

	void loadConfiguration(std::string name);

	void reconnectSim(/* representation */);

	void reconnectReal(/* representation */);

	bool ensureModuleTypeIsLoaded(std::string type);

	std::vector<ModuleInstance> modules;

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


int main (int argc, char **argv)
{
	ros::init(argc, argv, "module_manager");
	ros::start();

	ros::NodeHandle nh("~");

	ModuleManager modman;

	XmlRpc::XmlRpcValue moduleYaml;

	if(!ros::param::get("/modules", moduleYaml))
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

			if (modman.ensureModuleTypeIsLoaded(type))
				printf("module type is okay.\n");
			else
				printf("failed to load module type!\n");
		}

	}

	//std::string modules;
	//nh.param("/rosdistro", modules, std::string("not_set"));

	//printf("module setup was\n---------\n%s\n", modules.c_str());

	return 0;
}




















