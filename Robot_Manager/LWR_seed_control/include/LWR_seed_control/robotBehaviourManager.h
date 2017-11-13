#ifndef ROBOTBEHAVIOURMANAGER_H
#define ROBOTBEHAVIOURMANAGER_H

#include <ros/package.h>

#include "LWR_seed_control/robotControl.h"
#include "kuka_seed_commands/commandInterpreter.h"

#include <boost/serialization/map.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#define Z_AXIZ_OFFSET 0.05 // 15cm
#define OFFSET_ 0.15 // 3cm


class robotBehaviourManager
{
public:
    typedef enum {
        // Robot
        TASK_FINISHED  = 0, // Task correctly executed
        TASK_FAILED    = 1, // Task failed (errors occurred)
        TASK_EXECUTION = 2, // Robot is executing the task
        TASK_STOPPED   = 3, // Task stopped from user
        TASK_RESTART   = 4  // Task restarted from user
    } taskState;

    robotBehaviourManager( ros::NodeHandle node,
                           std::string objectListFile = "/data/objects_database_make_coffee.xml",
                           float posThreshold = 0.2,
                           float velThreshold = 0.05,
                           std::string commandTopicName = "kuka_action",
                           std::string taskStateTopicName = "kuka_return" );

    ~robotBehaviourManager();

    /**** Functions ****/
    // Create a thread specified in (thread_routine)
    void createThread( const unsigned int priority_,
                       const int schedulingPolicy_,
                       void *(*threadRoutine) (void *) ); // Start thread

    // Stop/Start task execution
    static void* taskControlThread(void* objectPointer);

    // Build and execute a task.
    static void* generateRobotBehaviour(void* objectPointer);

    void saveLearnedTask(std::string pathName);
    bool loadLearnedTask(std::string pathName);

    void saveObjectList(std::string fileName);

    /**** Members ****/
    int taskStatus;

    bool termintateTaskThread_;

private:
    /**** Functions ****/

    // Find the desired robot pose to grasp or release an object.
    int findRobotObjectPose( objectsList::objectsDescription object,
                             std::vector<double> &desPose,
                             bool releaseObject );

    int findRobotObjectPoseNew( objectsList::objectsDescription object,
                                std::vector<double> &desPose,
                                int actionIndex );

    tf::Transform stdVectorsToTfTranform( std::vector<double> position,
                                          std::vector<double> orientationYPR );

    // A routine to execute a single action
    int executeTaskRoutine(int objectIndex, int actionIndex);

    int executeRelativeReaching(int objectIndex, int actionIndex);
    int executeAbsoluteReaching(int objectIndex, int actionIndex);
    int executeDeltaReaching(int objectIndex, int actionIndex);
    int executeComplexAction(int objectIndex, int actionIndex);

    void splineDownSampling(vector< vector<double> > &desPos, double samplingTime);

    /**** Members ****/
    ros::NodeHandle behNode;

    commandInterpreter *commInterp;
    robotControl       *robCtrl;

    int oldCmdGenBeh, oldCmdCtrlTh;

    std::vector<double> desiredPose, showPosition, showOrientation;

    tf::TransformListener *tfListControl;
    tf::StampedTransform qrPose;

    // Action segmentation
    //bool isInTeachMode;
    float positionThresh, velocityThresh;

    // Thread members
    bool threadCreated_, taskThreadReady_;
    pthread_t	    taskThread_;
    pthread_mutex_t mutexVar_;
    pthread_cond_t  condVar_;
};

#endif // ROBOTBEHAVIOURMANAGER_H
