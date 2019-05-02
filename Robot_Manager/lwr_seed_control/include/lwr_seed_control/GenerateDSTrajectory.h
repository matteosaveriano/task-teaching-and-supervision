#ifndef GENERATEDSTRAJECTOY_H
#define GENERATEDSTRAJECTOY_H

#include <cstdlib>
#include <iostream>
#include <errno.h>
#include <math.h>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "lwr_seed_control/DMPData.h"

#include "lwr_seed_control/DMP/dmp.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float64MultiArray.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#define SEDS_LOAD_ERROR   -1
#define STATE_LOAD_ERROR  -2
#define LEN_OF_CART_FRAME 12 // To store less then 16 data
#define LEN_OF_CART_POS   3
#define LEN_OF_CART_ORI   3  // Minimal representation of orientation

using namespace std;

class GenerateDSTrajectoy
{
public:
    typedef enum {
        LIN_DS = 0,
        DMP    = 1
    } genMode;

    GenerateDSTrajectoy( vector<double> goalFrame,
                         vector<double> stiffness,
                         vector<double> normDamping,
                         double         sampleTime,
                         bool useOrientation = false,
                         unsigned int generationMode = 0 );

    ~GenerateDSTrajectoy(){}

    int update(vector<double> currFrame);
    int update(vector<double> currFrame, double currentTime);

    void GetNextFrameFloatArray(float *NextCartFrame);

    void ResetDSState( vector<double> goalFrame,
                       vector<double> stiffness,
                       vector<double> normDamping,
                       double sampleTime,
                       bool useOrientation = false,
                       unsigned int generationMode = 0);

    inline void SetGoal(vector<double> goal){ goalFrame_ = goal;
                                              // For DMP generation
                                              goalPosition(0) = goalFrame_[3];
                                              goalPosition(1) = goalFrame_[7];
                                              goalPosition(2) = goalFrame_[11];

                                              // Store goal rotation matrix (eigen format)
                                              ExtractRotationMat(goalFrame_, goalOrientation);
                                              //cout << goalOrientation << endl;
                                              // Store goal Euler angles
                                              goalEulerAngles = rotationMatrixToRPY(goalOrientation);  }

    inline void SetCurrEulerAngles(vector<double> frame){
        ExtractRotationMat(frame, currOrientation);

        currEulerAngles = rotationMatrixToRPY(currOrientation); //currOrientation.eulerAngles(2, 1, 0);

        // CHECK FOR SINGULARITIES +- 180deg
        double maxAngle = 90.0*3.1416/180.0;
        /*for(int i=0; i<3; ++i){
            if(goalEulerAngles(i)>maxAngle && currEulerAngles[i]<-maxAngle){
                goalEulerAngles(i) = -goalEulerAngles(i);
            }
            else if(goalEulerAngles(i)<-maxAngle && currEulerAngles[i]>maxAngle){
                goalEulerAngles(i) = -goalEulerAngles(i);
            }
        }*/

        for(int i=0; i<3; ++i){
            if(goalEulerAngles(i)>maxAngle && currEulerAngles[i]<-0.0){
                goalEulerAngles(i) = goalEulerAngles(i) - 2.0*3.1416;
            }
            else if(goalEulerAngles(i)<-maxAngle && currEulerAngles[i]>0.0){
                goalEulerAngles(i) = goalEulerAngles(i) + 2.0*3.1416;
            }
        }
        ///////////////////////////////////

        cout << "Current YPR: " << (180.0/3.1416)*currEulerAngles.transpose() << endl;
        cout << "Goal YPR: " << (180.0/3.1416)*goalEulerAngles.transpose() << endl;
       }

    void ExtractPositionVec(vector<double> omogTransfVec, vector<double> &posVec);
    void ExtractRotationMat(vector<double> omogTransfVec, Eigen::Matrix<double, 3, 3> &rotMat);

    // Train DMP, inputData and outputData are Nx3 matrices
    void trainDMP(vector<vector<double> > inputData, vector<double> times, bool position);
    inline void setTauDmp(double tau){tauDmp = tau;}
    inline double getTauDmp(){return tauDmp;}

    inline vector<double> getGoalFrame(){return goalFrame_;}
    void setGoalFrameDMP();

    void saveDMPparamsToFile(std::string fileName);
    bool loadDMPparamsFromFile(std::string fileName, int spaceDim);

    void FirstOrderDSNextPosition(vector<double> currFrame);

    inline Eigen::Vector3d getCurrEulerAngles(){return currEulerAngles;}

    inline void SetStiffness(vector<double> stiffness){stiffness_ = stiffness;}

    // Members
    vector<double> nextFrame_;
    vector<double> currLinVelocity_, currAngVelocity_;
    vector<double> goalFrame_;
    Eigen::Vector3d goalPosition;
    // Orientation related
    Eigen::Matrix<double, 3, 3> goalOrientation;
    Eigen::Vector3d goalEulerAngles;

    //Only diagonal Stiffness and Damping
    vector<double> stiffness_, origStiff_;
    vector<double> damping_;

    double sampleTime_;

    bool goalReached;

private:
    // Motion generation functions
    void FirstOrderLSTraj(vector<double> currFrame);

    void GaussianMixtureRegressionTraj(vector<double> currFrame);

    void GaussianProcessRegressionTraj(vector<double> currFrame, int iterNum);

    void DMPTrajectory(vector<double> currFrame, double currentTime);

    string numToString(int number);

    // THIS IS NORMALIZED DAMPING d = 2*nd*sqrt(stiff) SET THE STIFFNESS BEFORE
    void SetDamping(vector<double> normDamping);

    inline void SetSampleTime(double sampleTime){sampleTime_ = sampleTime;}

    inline void SetGoalReached(bool val_){goalReached = val_;}

    void SetPosOmogTranfVec(vector<double> &omogTransfVec, vector<double> posVec);
    void SetOriOmogTranfVec(vector<double> &omogTransfVec, vector<double> zyxAngles);

    void eulerToRotMatrix(vector<double> zyxAngles, Eigen::Matrix<double, 3, 3> &rotMat);

    bool computeOrientationLinDS(vector<double> currFrame);

    bool computePositionDMP(vector<double> currFrame, double currTime);
    bool computeOrientationDMP(vector<double> currFrame, double currTime);

    void computeNextStateDMP( std::vector<lwr_seed_control::DMPData> dmpList,
                              vector<double> currentState,
                              vector<double> goal,
                              vector<double> initialState,
                              double tau,
                              double currentTime,
                              vector<double> &nextState );

    void checkSingularitiesRPY(std::vector<double> oldAngles, std::vector<double> &newAngles);
    void checkSingularitiesRPY(Eigen::Vector3d oldAngles, std::vector<double> &newAngles);

    Eigen::Vector3d rotationMatrixToRPY(Eigen::Matrix<double, 3, 3> Rotation);

    int vectorMatrixToDmpTraj(std::vector< std::vector<double> > points, std::vector<double> times);

    // Members
    double endTrajCondition, endTrajOrientation;
    Eigen::Matrix<double, 3, 3> currOrientation;
    Eigen::Vector3d currEulerAngles;

    int genMode_;
    bool useOri_;

    // DMP objects
    lwr_seed_control::DMPTraj dmpInputData;
    std::vector<lwr_seed_control::DMPData> dmpPosition;
    std::vector<lwr_seed_control::DMPData> dmpOrientation;
    std::vector<double> kGainsPos, dGainsPos, kGainsOri, dGainsOri;
    std::vector<double> currDmpVelocity, currDmpAngVelocity, dmpInitPos, dmpInitOri;
    double dmpTime, tauDmp;
    int numBasesPos, numBasesOri;
};

#endif // GENERATEDSTRAJECTOY_H
