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
#include <geometry_msgs/TransformStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/PlanningScene.h>
#include <shape_msgs/Mesh.h>
#include <urdf/model.h>
#include <stdio.h>

class ModuleInstance;

class ModuleConnection
{
public:
	ModuleConnection()
	{
		joined = false;
		connectedToModule = NULL;
		connectedToConnector = 0;
	};

	bool joined;
	ModuleInstance *connectedToModule;
	int connectedToConnector;
};

class ModuleType
{
public:
	ModuleType()
	{
		name="Warning_name_not_set";
		loadedOkay = false;
		hasVisualModel = false;
		isRobotConnection = false;
	};
	~ModuleType()
	{
		if (hasCollisionModel)
			delete collisionMesh;
	}
	ModuleType(std::string typeName, std::string URDFString);

	bool fixedJointExists(urdf::Model *model, std::string linkName);

	tf2::Transform getFixedJointTF(boost::shared_ptr< const urdf::Joint > joint);

	std::string makeModuleFrameId(std::string moduleId);

	std::string name;

	bool loadedOkay;

	bool hasVisualModel;
	bool hasCollisionModel;

	shapes::Mesh *collisionMesh;

	bool isRobotConnection;

	urdf::Model urdf;

	// list of connectors for the module
	std::vector<tf2::Transform> connectors;

	std::string visualFileName;
	std::string collisionFileName;
};

ModuleType::ModuleType(std::string typeName, std::string URDFString)
{
	name = typeName;
	isRobotConnection = false;

	if (!urdf.initString(URDFString.c_str()))
	{
		ROS_ERROR("Failed to parse urdf string for module type [%s]", typeName.c_str());
		loadedOkay = false;
		hasVisualModel = false;
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
			hasVisualModel = true;
			//ROS_INFO("Read module visual filename of [%s]", visualFileName.c_str());
		}
		else
		{
			ROS_ERROR("Error no visual mesh file specified for module!");
			visualFileName = "NotSet";
			hasVisualModel = false;
		}

		// get the collision model of this module
		if (baseLink->collision->geometry->type == urdf::Geometry::MESH)
		{
			boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(baseLink->collision->geometry);

			collisionFileName = mesh->filename;
			hasCollisionModel = true;

			ROS_INFO("Reading module collision filename of [%s]", collisionFileName.c_str());
			collisionMesh = shapes::createMeshFromResource(collisionFileName);
		}
		else
		{
			ROS_ERROR("Error no collision mesh file specified for module!");
			collisionFileName = "NotSet";
			hasCollisionModel = false;
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
			//printf("made TF okay.\n"); fflush(stdout);
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

std::string ModuleType::makeModuleFrameId(std::string moduleId)
{
	return "module_" + moduleId + "_tf";
}

class ModuleTypeArm : public ModuleType
{
public:
	ModuleTypeArm()
	{
		name = "arm";
		loadedOkay = true;
		hasVisualModel = false;
		isRobotConnection = true;

		// create a single 180 degree rotation connection to link the arm_base_link_frame to the connector
		tf2::Transform connector;
		connector.setIdentity();
		tf2::Quaternion z180;
		z180.setRPY(0.0, 0.0, 3.141592654);
		connector.setRotation(z180);
		connectors.push_back(connector);
	};

	// override makeModuleFrameId method so it always returns base_link
	//std::string makeModuleFrameId(std::string moduleId)
	//{
	//	return "base_link";
	//};
};

class ModuleInstance
{
public:

	ModuleInstance(ModuleType *modType, std::string id)
	{
		type = modType;
		this->id = id;
		poseRelativeToRootKnown = false;

		// create a vector of open connection objects for each connector
		connections.resize(type->connectors.size());

		// create 180 degree z rotation TF used to calculate joined connector transforms
		connectorRotation.setIdentity();
		tf2::Quaternion z180;
		z180.setRPY(0.0, 0.0, 3.141592654);
		connectorRotation.setRotation(z180);
	}

	/// Method to attempt to join two connectors together
	/*
	 * Method to attempt to join two connectors together,verifies that both connectors are
	 * initially free then creates a reciprocal link between them
	 */
	bool joinConnector(unsigned int thisConnectorIdx, ModuleInstance *otherModule, unsigned int otherConnectorIdx)
	{
		// verify this is a valid joint to make
		if (thisConnectorIdx >= (int)connections.size())
		{
			ROS_ERROR("Error trying to join connectors: This connector index (%d) out of range (0 - %d)!",
					  thisConnectorIdx,
					  (int)(connections.size())-1);
			return false;
		}
		if (connections[thisConnectorIdx].joined)
		{
			ROS_ERROR("Error trying to join connectors: This connector (%d) is already joined!", thisConnectorIdx);
			return false;
		}
		if (otherConnectorIdx >= (int)otherModule->connections.size())
		{
			ROS_ERROR("Error trying to join connectors: Other connector index (%d) out of range (0 - %d)!",
					  otherConnectorIdx,
					  (int)(otherModule->connections.size())-1);
			return false;
		}
		if (otherModule->connections[otherConnectorIdx].joined)
		{
			ROS_ERROR("Error trying to join connectors: Other connector (%d) is already joined!", otherConnectorIdx);
			return false;
		}

		// make this side of the joint
		connections[thisConnectorIdx].joined = true;
		connections[thisConnectorIdx].connectedToModule = otherModule;
		connections[thisConnectorIdx].connectedToConnector = otherConnectorIdx;

		// make the other size of the joint
		otherModule->connections[otherConnectorIdx].joined = true;
		otherModule->connections[otherConnectorIdx].connectedToModule = this;
		otherModule->connections[otherConnectorIdx].connectedToConnector = thisConnectorIdx;

		return true;
	}

	/// Method to calculate the relative pose of the connected module in the frame of this module via connected joints
	tf2::Transform getJointTF(unsigned int connectionIdx)
	{
		tf2::Transform identityTF;
		identityTF.setIdentity();

		if (connectionIdx > (int)connections.size())
		{
			ROS_ERROR("Error getting joint TF: connection index (%d) out of range (0 - %d)",
					  connectionIdx,
					  (int)connections.size()-1);
			return identityTF;
		}
		if (!connections[connectionIdx].joined)
		{
			ROS_ERROR("Error getting joint TF: connection index (%d) current open!",
					  connectionIdx);
			return identityTF;
		}

		int childConnectorIdx = connections[connectionIdx].connectedToConnector;
		ModuleType *childModuleType = connections[connectionIdx].connectedToModule->type;

		tf2::Transform parentTF = type->connectors[connectionIdx];
		tf2::Transform childTF = childModuleType->connectors[childConnectorIdx];


		tf2::Transform jointTF = (parentTF * connectorRotation) * childTF.inverse();
		return jointTF;
	}

	std::string makeModuleFrameId();

	ModuleType *type;
	std::string id;

	/// the pre-calculated pose of this module relative to the rood modules it is static relative to
	tf2::Transform poseRelativeToRoot;
	bool poseRelativeToRootKnown;

	/// rootFrameId will be either the bus_frame or a robots EE frame.
	std::string rootFrameId;

	/// List of connection status objects for each connector of this module
	std::vector<ModuleConnection> connections;

	tf2::Transform connectorRotation;
};

std::string ModuleInstance::makeModuleFrameId()
{
	return type->makeModuleFrameId(id);
}

class ModuleManager
{
public:

	ModuleManager(std::string planningScene = "None") : rootSet(false), modulePosesUptoDate(false)
	{
		ros::NodeHandle n;
		moduleMarkerPub = n.advertise<visualization_msgs::MarkerArray>("module", 10);

		// create robot base and end effector virtual modules so they can be referenced by concrete modules as they're loaded
		loadedModuleTypes.emplace("arm", ModuleTypeArm());

		// if a planning scene was given then attach to it
		planningSceneAttached = (planningScene != "None");
		if (planningSceneAttached)
			planning_scene_diff_pub = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	};

	ModuleInstance* getRootModule();

	bool validateModuleStructure();

	void publishTFs();

	void publishModuleMarkers();

	void updateMoveitCollisionScene(bool first = false);

	bool loadConfiguration(std::string name);

	void reconnectSim(/* representation */);

	void reconnectReal(/* representation */);

	bool ensureModuleTypeIsLoaded(std::string type);

	ModuleType *getModuleType(std::string type);

	ModuleInstance *getInstanceById(std::string id);

private:

	/// Helper function to form the module frame id strings
	std::string makeModuleFrameId(std::string moduleId);

	/// Helper function to refresh the caches poses of all the modules
	void updateModulePoses();

	/// Helper function to convert a tf2_transform object to a geometry_msgs::Transform message
	geometry_msgs::Transform transformToTransformMsg(tf2::Transform transform);

	/// vector of module instances
	std::vector<ModuleInstance> modules;
	bool modulePosesUptoDate;

	std::string rootModuleId;
	bool rootSet;

	/// vector of module types currently loaded by the system
	std::map<std::string, ModuleType> loadedModuleTypes;

	bool planningSceneAttached;

	ros::Publisher moduleMarkerPub;
	ros::Publisher planning_scene_diff_pub;
};

void ModuleManager::publishTFs()
{
	static tf2_ros::TransformBroadcaster br;

	ros::Time timePoint = ros::Time::now();

	if(!modulePosesUptoDate)
		updateModulePoses();

	for (int m=0; m<modules.size(); ++m)
	{
		// if the relative pose of this module is not known.
		// this will only happen in error cases but the error is reported by the
		// updateModulePoses method, not here.
		if (!modules[m].poseRelativeToRootKnown)
			continue;

		// if this is not the root module then publish it's TF relative to it's root node.
		if (modules[m].id != rootModuleId)
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
	//std::string busRootFrameId = makeModuleFrameId(rootModule->id);
	std::string manipulatorEEFrameId = "ee_frame";

	// reset known module positions except for roots (sat bus and robot EE) TODO (Update this)
	for (int m=0; m<modules.size(); ++m)
	{
		modules[m].poseRelativeToRootKnown = (modules[m].id == rootModuleId);
		if (modules[m].poseRelativeToRootKnown)
		{
			modules[m].rootFrameId = makeModuleFrameId(modules[m].id);
			modules[m].poseRelativeToRoot.setIdentity();
		}

		printf("Module [%d] \"%s\" ", m, modules[m].id.c_str());
		if (modules[m].poseRelativeToRootKnown)
			printf("Is root");
		else
			printf("Is not root");
		printf(" and has a rootFrameId of \"%s\"\n", modules[m].rootFrameId.c_str());
	}

	// repeatedly loop through the modules adding the known positions of new modules
	// if their direct parents are known.
	// stop looping if no more modules are known by a single iteration
	int modulesAdded = 1;
	int passCount = 0;
	while(modulesAdded > 0)
	{
		modulesAdded = 0;

		for (int m=0; m<modules.size(); ++m)
		{
			// if the pose of this module is already known then skip it
			if (modules[m].poseRelativeToRootKnown == true)
				continue;

			std::vector<std::string> connectionChains;
			std::vector<tf2::Transform> poseTFs;
			std::vector<std::string> connectionRootIds;

			// find any connections between this module and a module of known position
			// and calculate the pose of this module via each connection.
			for (int c=0; c<modules[m].connections.size(); ++c)
				if (modules[m].connections[c].joined)
				{
					ModuleInstance *otherModule = modules[m].connections[c].connectedToModule;
					if (otherModule->poseRelativeToRootKnown)
					{
						// format connection chain, used for debugging and error reporting of inconsistencies
						std::string chain = "Parent \"" + modules[m].id +
											"\"[" + std::to_string(c) +
											"] -> Child \"" + otherModule->id +
											"\"[" + std::to_string(modules[m].connections[c].connectedToConnector) +
											"]";
						connectionChains.push_back(chain);
						tf2::Transform connectionTF = modules[m].getJointTF(c);
						tf2::Transform poseTF = otherModule->poseRelativeToRoot * connectionTF.inverse();
						poseTFs.push_back(poseTF);
						connectionRootIds.push_back(otherModule->rootFrameId);
					}
				}

			ROS_WARN("Module \"%s\" Found %d connections to placed modules", modules[m].id.c_str(), (int)poseTFs.size());
			for (int d=0; d<poseTFs.size(); ++d)
			{
				ROS_WARN("Found connection (%s)", connectionChains[d].c_str());
				ROS_WARN("Quaternion (%f %f %f %f)", poseTFs[d].getRotation().getX(),
													 poseTFs[d].getRotation().getY(),
													 poseTFs[d].getRotation().getZ(),
													 poseTFs[d].getRotation().getW());
			}

			// if there were no connections then don't do anything
			if (poseTFs.size() == 0)
				continue;

			// if there were more than 1 connections then verify each transform is consistent within machine epislon
			if (poseTFs.size() > 1)
			{
				// TODO
			}

			// set the pose of this module
			modules[m].poseRelativeToRoot = poseTFs[0];
			modules[m].poseRelativeToRootKnown = true;
			modules[m].rootFrameId = connectionRootIds[0];
			++modulesAdded;

			// update the known pose of this module

			/*tf2::Transform transform;
			transform.setOrigin( tf2::Vector3(m*1.0, m*0.5, m*0.2) );
			tf2::Quaternion q;
			q.setRPY(0, 0, m);
			transform.setRotation(q);

			// define the pose of the module
			modules[m].poseRelativeToRoot = transform;

			// define what this pose is relative to
			modules[m].rootFrameId = busRootFrameId;*/
		}

		ROS_INFO("Pose Calculation pass %d, placed %d new modules.", passCount, modulesAdded);
		++passCount;
	}

	// check if there are any modules with unknown poses (i.e. disconnected modules)
	int looseModuleCount = 0;
	for (int m=0; m<modules.size(); ++m)
	{
		if (!modules[m].poseRelativeToRootKnown)
			++looseModuleCount;
	}
	if (looseModuleCount > 0)
		ROS_ERROR("Error finding poses of all modules: %d of %d modules could not be located!",
				  looseModuleCount,
				  (int)modules.size());

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
		// if this module doesn't have a visual model then skip it
		if (!modules[m].type->hasVisualModel)
			continue;

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
		marker.color.a = 1.0;
		if (modules[m].id == rootModuleId)
		{
			marker.color.r = 1.0;
			marker.color.g = 1.0;
			marker.color.b = 1.0;
		}
		else
		{
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
		}
		marker.mesh_resource = modules[m].type->visualFileName;

		ma.markers.push_back(marker);
	}

	moduleMarkerPub.publish(ma);
}

void ModuleManager::updateMoveitCollisionScene(bool first)
{
	// check there is an attached moveGroupInterface
	if (!planningSceneAttached)
	{
		ROS_ERROR("Error call to updateMoveitCollisionScene on a ModuleManager which is not connected to a planning scene!");
		return;
	}

	// the planningScene message to add all the collision scene updates into
	moveit_msgs::PlanningScene planningSceneUpdate;

	planningSceneUpdate.is_diff = true;

	// loop through all modules adding/updating them in the collision scene
	for (int m=0; m<modules.size(); ++m)
	{
		// if this module doesn't have a collision model then skip it
		if (!modules[m].type->hasCollisionModel)
			continue;

		moveit_msgs::AttachedCollisionObject attachedModule;
		attachedModule.link_name = "r_wrist_roll_link";  // <---      TODO
		/* The header must contain a valid TF frame*/
		attachedModule.object.header.frame_id = modules[m].makeModuleFrameId();
		/* The id of the object */
		attachedModule.object.id = "collision" + modules[m].makeModuleFrameId();

		// Identity pose because module it always at it's own origin
		geometry_msgs::Pose pose;
		pose.orientation.w = 1.0;

		// attach pose and collision mesh to message
		shapes::ShapeMsg collisionMesh;
		shapes::constructMsgFromShape(modules[m].type->collisionMesh, collisionMesh);
		attachedModule.object.meshes.push_back(boost::get<shape_msgs::Mesh>(collisionMesh));
		attachedModule.object.mesh_poses.push_back(pose);

		attachedModule.object.operation = attachedModule.object.ADD;

		// TODO need to add a check to see if this module is relative to the Sat Bus or robot EE and update
		// planningSceneUpdate.world....
		// or
		// PlanningSceneUpdate.robot.... acordingly

		// add this object diff to the update message
		planningSceneUpdate.world.collision_objects.push_back(attachedModule.object);
	}
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

/// Method to find an instance with the given id and return a ptr to the module instance.
/*
 * Returns NULL if no matching module was found
 */
ModuleInstance *ModuleManager::getInstanceById(std::string id)
{
	for (int m=0; m<modules.size(); ++m)
	{
		if (modules[m].id == id)
			return &modules[m];
	}

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
		return false;
	}

	if (moduleYaml.getType() != XmlRpc::XmlRpcValue::TypeArray)
	{
		ROS_ERROR("Error loading initial configuration yaml: 'modules' is not an array!");
		return false;
	}

	// 1st pass over module list to load all references URDF definitions and create module instances
	for (int i=0; i<moduleYaml.size(); ++i)
		if (moduleYaml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
		{

			std::string id = moduleYaml[i]["id"];
			std::string type = moduleYaml[i]["type"];
			bool isRoot = moduleYaml[i]["root"];

			if (id != "" && type != "")
			{
				if (ensureModuleTypeIsLoaded(type))
				{
					ModuleInstance newModule(getModuleType(type), id);
					modules.push_back(newModule);

					// if this is the root module then add a link to.
					if (isRoot)
						//rootModule = &(modules[modules.size()-1]);
						rootModuleId = newModule.id;
				}
				else
					ROS_ERROR("Error loading initial configuration yaml: Failed to load module type \"%s\"!", type.c_str());
			}
			else
			{
				ROS_ERROR("Reading initial module config: Id or Type not set when reading module instance!");
			}
		}
	ROS_INFO("Created %d module instances.", (int)modules.size());

	// 2nd pass over module list to create joints between connectors
	int connectorCount = 0;
	for (int i=0; i<moduleYaml.size(); ++i)
		if (moduleYaml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
		{

			// if this module instance has a connections list
			if (moduleYaml[i].hasMember("connections"))
			{
				std::string id = moduleYaml[i]["id"];
				ModuleInstance *module = getInstanceById(id);
				XmlRpc::XmlRpcValue connections = moduleYaml[i]["connections"];

				for (int c=0; c<connections.size(); ++c)
				{
					std::string childModuleId = connections[c]["child_module"];
					int childConnectorIdx = connections[c]["child_connector"];
					int parentConnectorIdx = connections[c]["parent_connector"];

					ModuleInstance *childModule = getInstanceById(childModuleId);
					if (childModule == NULL)
					{
						ROS_ERROR("Error reading initial module connections: Cannot find child module with id \"%s\"",
								  childModuleId.c_str());
						return false;
					}
					else
					{
						ROS_WARN("About to join module \"%s\"[%d] to module \"%s\"[%d]",
								 id.c_str(), parentConnectorIdx,
								 childModuleId.c_str(),  childConnectorIdx);
						module->joinConnector(parentConnectorIdx, childModule, childConnectorIdx);
						++connectorCount;
					}
				}
			}
		}
	ROS_INFO("Created %d connector joints.", connectorCount);

	return true;
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




















