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

class ModuleConnector
{
	tf::Transform pose;
};

class ModuleType
{
public:

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

	ModuleManager();

	ModuleInstance* getRootModule();

	bool validateModuleStructure();

	void publishTransforms();

	void updateMoveitCollisionScene();

	void loadConfiguration(std::string name);

	void reconnectSim(/* representation */);

	void reconnectReal(/* representation */);

	std::vector<ModuleInstance> modules;
};


int main (int argc, char **argv)
{
	return 0;
}
