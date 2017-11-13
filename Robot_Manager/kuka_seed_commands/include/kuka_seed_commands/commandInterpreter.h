#ifndef COMMANDINTERPRETER_H
#define COMMANDINTERPRETER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <pthread.h>

#include "kuka_seed_commands/objectsList.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#define DEBUG_MODE
#define POS_THRESH 0.14
#define CUP_THRESH 0.3
#define ACTION_OFFSET 0.2


// A class to define robot and gripper behaviours according
// to the output from the UNINA's multimodal architecture
class commandInterpreter
{
public:
    typedef enum {
        // Robot
        // Terminate execution
        CMD_TERMIN  = -2, // Close all active threads
        CMD_STOP    = 0, // Stop the robot in current position
        CMD_START   = 1, // ?
        CMD_EXECUTE = 2, // Execute one signle action
        CMD_TEACH   = 3, // Robot in gravity compensation
        CMD_PLACE  = 4, // Store a tool
        CMD_TAKE    = 5, // Take a tool
        CMD_DONE   = 6, //  Teaching finished
        CMD_SHOW    = 7, // Show a tool
        // Gripper
        CMD_CLOSE   = 8, // Grasp object and store grasping pose
        CMD_OPEN    = 9 // Release object and store release pose
    } integrationCommands;

    commandInterpreter( ros::NodeHandle node,
                        std::string objectListFile   = "/data/objects_database_make_tea.xml",
                        std::string seedToKukaTopic = "kuka_action",
                        std::string kukaToSeedTopic = "kuka_return",
                        std::string objectDistTopic = "object_dist",
                        std::string gripperTopic = "wsg_50_control/cmd_gripper",
                        std::string objectsTopic = "ar_pose_marker");

    ~commandInterpreter(){std::cout << "destroy commandInterpreter objects" << std::endl; delete objects; std::cout << "commandInterpreter objects destroyed" << std::endl;delete tfListener;}

    void gripperControl(std_msgs::Int32 gripperCommand_);

    // Read tf transformation
    int waitTfTransform( std::string targetFrame,
                         std::string sourceFrame,
                         tf::StampedTransform &transform );

    std::string numToString(int number);

    /**** Members ****/
    int robotCommand, gripperCommand, objectToUseId, objectToUseIndex, learnObjIndx;
    int closestObjId, actionToPerformIndex;
    double closestObjectDist;

    bool taskExecuted;
    bool closeCmdReceived, openCmdReceived;
    bool preGraspPoseSetted;
    bool setPreGraspPose;
    bool isTeachMode;

    objectsList *objects;

    //std::string arFrameName;

    // Thread (callbacks) lock and sleep
    pthread_mutex_t callbackMtx;

private:
    /**** Functions ****/
    // Read seed command from ros topic
    void seedToKukaCallback(const std_msgs::String::ConstPtr &cmdPtr);

    // Read tracked objects from ar_track_alvar node and update objectList
    void objTrackingCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &objPtr);

    // Read object name from ros topic (speech recognition module)
    void objNameCallback(const std_msgs::StringConstPtr &namePtr);

    // Update object after learning grasp and release pose
    void updateLearnedObject(bool verbouse_ = false);

    // Find the object closest (Euclidean distance) to the robot
    int findClosestObject(int &objectIndex);

    // Find the tray closest (Euclidean distance) to the robot
    //int findClosestTray(int &objectIndex);

    int setGrasingPose(std::string arFrameName);

    int getCurrentPose(tf::StampedTransform &tmpPose);

    int checkGripperAlignment(double &offsetDign);

    void checkSingularitiesRPY(std::vector<double> oldAngles, Eigen::Vector3d &newAngles);

    std::vector<std::string> seedCommandParser(std::string schemaInstance);

    Eigen::Vector3d rotationMatrixToRPY(Eigen::Matrix<double, 3, 3> Rotation);

    Eigen::Vector3d tfStampedTransformToRPY(tf::StampedTransform transform);

    /**** Members ****/
    ros::NodeHandle cmdNode;
    ros::Publisher grippCommPub, kukaToSeedPub, objectDistPub;
    ros::Subscriber seedToKukaSub, arObjSub;
    //std::string seedToKukaTopic_, kukaToSeedTopic_, objectDistTopic_, gripperTopic_;
    std::string lastSeedCommand, newSeedCommand;

    // tf read
    ros::Time now, prev;
    tf::TransformListener *tfListener;
    tf::StampedTransform graspingPose, releasePose;
    std::string objId;

    int lastSegmentID;

    bool publishMessage;

    // New object
    objectsList::objectsDescription object;
    std::string newObjectFile, newObjectName;

    // New Action
    objectsList::actionDescriptor newAction;

    int counterNewObject;

    int cupCloseCounter;
    bool closeToCup, complexActionFound;
};

#endif // COMMANDINTERPRETER_H
