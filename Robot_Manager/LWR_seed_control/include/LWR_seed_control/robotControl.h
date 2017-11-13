#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <math.h>
#include <string>
#include <errno.h>
#include <map>
#include <pthread.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>

#include "LWR_seed_control/GenerateDSTrajectory.h"
#include "LWR_seed_control/spline.h"

#ifndef CART_FRM_DIM
#define CART_FRM_DIM 12
#endif

#ifndef LEN_OF_CART_POS
#define LEN_OF_CART_POS 6
#endif

#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif

class robotControl
{
public:
    // Constructor
    robotControl(ros::NodeHandle node, double samplingTime = 0.005);
    
    ~robotControl();

    /**** Functions ****/

    // Go to Cartesian Pose with linear DS
    int goToCartPose(vector<double> goalFrame);

    // Go to Gravity Compensation until executeBehaviour is true
    int goToGravityCompensation();

    // Go to goal DMP
    int goToCartPoseDMP(vector<double> goalFrame, int dmpActionIndex);

    // Rotation Matrix to RPY
    Eigen::Vector3d rotationMatrixToRPY(Eigen::Matrix<double, 3, 3> Rotation);

    std::vector<double> currentPose;

    // Execution control (stop from outside)
    pthread_mutex_t executionMtx;
    bool executeBehaviour;
    bool stopExecution;

    std::vector<GenerateDSTrajectoy> complexTraj;
   //std::vector<double> dmpK, dmpD;
    //GenerateDSTrajectoy *emptyDmp;
    std::map<std::string, int> actionIndexMap;
    vector<double> dmpStiffness, dmpDamping;

    void saveVectorMatrixToFile (string fileName, vector < vector <double> > outMat);

    bool getGripperPoseTf(std::vector<double> &pose);

    inline double getSamplingTime(){return samplingTime_;}
private:
    /**** Functions ****/
    //void initJointStateMsg();
    void publishGripperPoseTf(std::vector<double> pose);

    // Generate end-effector trajectory using Dynamical Systems
    GenerateDSTrajectoy *dsTraj;

    // Publish joint angles for \tf transformations
    ros::NodeHandle node_;
    sensor_msgs::JointState jointStateMsg;
    ros::Publisher jointStatePub;
    float commCartPose[CART_FRM_DIM];
    int seqCounter;
    double samplingTime_;

    tf::TransformListener *tfListener;
    ros::Publisher tfPub;
    tf::tfMessage tfMsg;

    // Thread members
    bool threadCreated_, taskThreadReady_;
    pthread_t	    taskThread_;
    pthread_cond_t  condVar_;
};

#endif // ROBOTCONTROL_H
