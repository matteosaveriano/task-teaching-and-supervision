#include "LWR_seed_control/GenerateDSTrajectory.h"

GenerateDSTrajectoy::GenerateDSTrajectoy( vector<double> goalFrame,
                                          vector<double> stiffness,
                                          vector<double> normDamping,
                                          double sampleTime,
                                          bool useOrientation,
                                          unsigned int generationMode):nextFrame_(goalFrame),
		                                                       currLinVelocity_(LEN_OF_CART_POS, 0.0),
		                                                       currAngVelocity_(LEN_OF_CART_POS, 0.0),
		                                                       goalFrame_(goalFrame),
		                                                       stiffness_(stiffness),
		                                                       origStiff_(stiffness),
		                                                       damping_(stiffness.size(), 0.0)
{
    SetSampleTime(sampleTime);

    endTrajCondition = 0.001*0.001;
    endTrajOrientation = 0.02*0.02; // 1 deg
    genMode_         = generationMode;
    useOri_          = useOrientation;

    SetGoalReached(false);

    //stiffness_[3] /= 3.0;
    //stiffness_[4] /= 3.0;
    //stiffness_[5] /= 3.0;

    unsigned int i = 0;
    for(i = 0; i < stiffness_.size(); ++i){
        damping_[i] = 2.0*normDamping[i]*sqrt(stiffness_[i]);
    }

    // Store goal position
    goalPosition(0) = goalFrame_[3];
    goalPosition(1) = goalFrame_[7];
    goalPosition(2) = goalFrame_[11];

    // Store goal rotation matrix (eigen format)
    ExtractRotationMat(goalFrame_, goalOrientation);

    // Store goal Euler angles
    goalEulerAngles = goalOrientation.eulerAngles(2, 1, 0);
// Init DMP
    if(genMode_ == GenerateDSTrajectoy::DMP){
        for(int i=0; i<3; ++i){
            kGainsPos.push_back(stiffness[i]);
            dGainsPos.push_back(normDamping[i]);
            currDmpVelocity.push_back(0.0);
            dmpInitPos.push_back(0.0);
            if(useOri_){
                dmpInitOri.push_back(0.0);
                kGainsOri.push_back(stiffness[i]);
                dGainsOri.push_back(normDamping[i]);
                currDmpAngVelocity.push_back(0.0);
            }
        }

        numBasesPos = numBasesOri = 15;
        tauDmp = 1.0;
    }
}


int GenerateDSTrajectoy::update(vector<double> currFrame){
    if(genMode_ == GenerateDSTrajectoy::LIN_DS){
        FirstOrderLSTraj(currFrame);
    }
    else if(genMode_ == GenerateDSTrajectoy::DMP){
        std::cout << "Please use: update(vector<double> currFrame, double currentTime) for DMP" << std::endl;
        return -1;
    }
    return 0;
}


int GenerateDSTrajectoy::update(vector<double> currFrame, double currentTime){
    if(genMode_ == GenerateDSTrajectoy::LIN_DS){
        std::cout << "Please use: update(vector<double> currFrame) for Linear DS" << std::endl;
        return -1;
    }
    else if(genMode_ == GenerateDSTrajectoy::DMP){
        DMPTrajectory(currFrame, currentTime);
    }

    return 0;
}


void GenerateDSTrajectoy::trainDMP(vector< vector<double> > inputData, vector<double> times, bool position){
    vectorMatrixToDmpTraj(inputData, times);

    if(position){
        DMP_RosWrapper::learnFromDemo(dmpInputData, kGainsPos, dGainsPos, numBasesPos, dmpPosition);
        size_t dim_ = inputData.size();
        goalPosition(0) = inputData[dim_-1][0];
        goalPosition(1) = inputData[dim_-1][1];
        goalPosition(2) = inputData[dim_-1][2];
        std::cout << "DMP goal position: " << goalPosition.transpose() << std::endl;
    }
    else{
        DMP_RosWrapper::learnFromDemo(dmpInputData, kGainsOri, dGainsOri, numBasesOri, dmpOrientation);
        size_t dim_ = inputData.size();
        goalEulerAngles(0) = inputData[dim_-1][0];
        goalEulerAngles(1) = inputData[dim_-1][1];
        goalEulerAngles(2) = inputData[dim_-1][2];
        std::cout << "DMP goal orientation: " << (180/3.14)*goalEulerAngles.transpose() << std::endl;
    }
}


string GenerateDSTrajectoy::numToString(int number){
  std::stringstream s;
  s << number;
  return s.str();
}


bool GenerateDSTrajectoy::loadDMPparamsFromFile(std::string fileName, int spaceDim){
    std::vector<std::string> topics;
    topics.push_back(std::string("dmp_pos"));
    topics.push_back(std::string("dmp_ori"));
    topics.push_back(std::string("dmp_tau"));
    topics.push_back(std::string("dmp_goal"));

    LWR_seed_control::DMPData *curr_dmp = new LWR_seed_control::DMPData();

    // Read DMP parameters
    for(int i=0; i<spaceDim; ++i){
        // Position
       // cout << fileName + "_pos_" + numToString(i) + ".bag" << endl;
        rosbag::Bag bagPos;
        bagPos.open(fileName + "_pos_" + numToString(i) + ".bag", rosbag::bagmode::Read);
        rosbag::View viewPos(bagPos, rosbag::TopicQuery(topics[0]));

        foreach(rosbag::MessageInstance const m, viewPos){
            LWR_seed_control::DMPData::ConstPtr s = m.instantiate<LWR_seed_control::DMPData>();
            if (s != NULL){
                curr_dmp->weights = s->weights;
                curr_dmp->k_gain = s->k_gain;
                curr_dmp->d_gain = s->d_gain;
                curr_dmp->f_domain = s->f_domain;
                curr_dmp->f_targets = s->f_targets;

                /*for(int t=0; t<curr_dmp->f_targets.size(); ++t)
                    cout << curr_dmp->f_targets[t] << " " << s->f_targets[t];

                cout << endl;*/

                dmpPosition.push_back(*curr_dmp);

         //       cout << dmpPosition[i].k_gain << endl;
            }
            else{
                ROS_ERROR("Cannot read data in %s \n", (fileName + "_pos_" + numToString(i) + ".bag").c_str());
                bagPos.close();
                return false;
            }
        }
        bagPos.close();

        // Orientation
        if(useOri_){
           // cout << fileName + "_ori_" + numToString(i) + ".bag" << endl;
            rosbag::Bag bagOri;
            bagOri.open(fileName + "_ori_" + numToString(i) + ".bag", rosbag::bagmode::Read);
            rosbag::View viewOri(bagOri, rosbag::TopicQuery(topics[1]));

            foreach(rosbag::MessageInstance const m, viewOri){
                LWR_seed_control::DMPData::ConstPtr s = m.instantiate<LWR_seed_control::DMPData>();
                if (s != NULL){
                    curr_dmp->weights = s->weights;
                    curr_dmp->k_gain = s->k_gain;
                    curr_dmp->d_gain = s->d_gain;
                    curr_dmp->f_domain = s->f_domain;
                    curr_dmp->f_targets = s->f_targets;

                    dmpOrientation.push_back(*curr_dmp);

                  //  cout << dmpOrientation[i].d_gain << endl;
                }
                else{
                    ROS_ERROR("Cannot read data in %s \n", (fileName + "_pos_" + numToString(i) + ".bag").c_str());
                    bagOri.close();
                    return false;
                }
            }
            bagOri.close();
        }
    }

    // DMP tau
    rosbag::Bag bagTau;
    bagTau.open(fileName + "_pos_" + numToString(0) + ".bag", rosbag::bagmode::Read);
    rosbag::View viewTau(bagTau, rosbag::TopicQuery(topics[2]));
    foreach(rosbag::MessageInstance const m, viewTau){
        std_msgs::Float64::ConstPtr s = m.instantiate<std_msgs::Float64>();
        if (s != NULL){
            tauDmp = s->data;
          //  cout << tauDmp << endl;
        }
        else{
            ROS_ERROR("Cannot read data in %s \n", (fileName + "_pos_" + numToString(0) + ".bag").c_str());
            bagTau.close();
            return false;
        }
    }
    bagTau.close();

    // DMP goal
    rosbag::Bag bagGoal;
    bagGoal.open(fileName + "_pos_" + numToString(0) + ".bag", rosbag::bagmode::Read);
    rosbag::View viewGoal(bagGoal, rosbag::TopicQuery(topics[3]));
    foreach(rosbag::MessageInstance const m, viewGoal){
        std_msgs::Float64MultiArray::ConstPtr s = m.instantiate<std_msgs::Float64MultiArray>();
        if (s != NULL){
            vector<double> goal(12, 0.0);
            for(int i=0; i<12; ++i){
                goal[i] = s->data[i];
                //cout << goal[i] << "\t";
            }
            cout << goal[3] << "\t" << goal[7] << "\t" << goal[11] << endl;
            SetGoal(goal);
        }
        else{
            ROS_ERROR("Cannot read data in %s \n", (fileName + "_pos_" + numToString(0) + ".bag").c_str());
            bagGoal.close();
            return false;
        }
    }

    bagGoal.close();
    return true;
}


void GenerateDSTrajectoy::saveDMPparamsToFile(std::string fileName){
    rosbag::Bag bag;

    ros::Time::init();
    ros::Time t_ = ros::Time::now();

   /* DMP_RosWrapper::DMPData *curr_dmp = new DMP_RosWrapper::DMPData();
    curr_dmp->weights.push_back(1.0);
    curr_dmp->k_gain = -1.0;
    curr_dmp->d_gain = 1.0;
    for(int i=0; i<10; i++){
                curr_dmp->f_domain.push_back(2.1);
                curr_dmp->f_targets.push_back(-11.0);
    }
    dmpPosition.push_back(*curr_dmp);

    for(int i=0; i<10; i++){
        curr_dmp->f_domain[i] = (2.1);
                curr_dmp->f_targets[i] = (3.0);
    }
    dmpPosition.push_back(*curr_dmp);

    for(int i=0; i<10; i++){
        curr_dmp->f_domain[i] = (2.1);
                curr_dmp->f_targets[i] = (0.0);
    }
    dmpPosition.push_back(*curr_dmp);

    dmpOrientation.push_back(*curr_dmp);
    dmpOrientation.push_back(*curr_dmp);
    dmpOrientation.push_back(*curr_dmp);*/


    cout << "DMP f: " << dmpPosition[0].f_targets.size() << endl;
    for(int i=0; i<(int)dmpPosition.size(); ++i){
        // Position
        bag.open(fileName + "_pos_" + numToString(i) + ".bag", rosbag::bagmode::Write);
        bag.write("dmp_pos", t_, dmpPosition[i]);

        // Save dmp tau
        std_msgs::Float64 tau_;
        tau_.data = tauDmp;
        bag.write("dmp_tau", t_, tau_);

        // Save goal
        std_msgs::Float64MultiArray goal_; //declare Atest
        goal_.data.resize(12); //resize the array to assign to existent values
        for(int f=0; f<12; ++f){
            goal_.data[f] = goalFrame_[f];
        }
        bag.write("dmp_goal", t_, goal_);

        bag.close();
    }

    if(useOri_){
        for(int i=0; i<(int)dmpOrientation.size(); ++i){
            bag.open(fileName + "_ori_" + numToString(i) + ".bag", rosbag::bagmode::Write);
            bag.write("dmp_ori", t_, dmpOrientation[i]);

            bag.close();
        }
    }
}


void GenerateDSTrajectoy::setGoalFrameDMP(){
    // Goal position
    goalFrame_[3] = goalPosition(0);
    goalFrame_[7] = goalPosition(1);
    goalFrame_[11] = goalPosition(2);

    // Orientation
    vector<double> zyxAngles(3, 0.0);
    zyxAngles[0] = goalEulerAngles(0);
    zyxAngles[1] = goalEulerAngles(1);
    zyxAngles[2] = goalEulerAngles(2);

    eulerToRotMatrix(zyxAngles, goalOrientation);

    SetOriOmogTranfVec(goalFrame_, zyxAngles);

}


int GenerateDSTrajectoy::vectorMatrixToDmpTraj(std::vector< std::vector<double> > points, std::vector<double> times){
    if(times.size() != points.size()){
        std::cout << "Unable to set DMP training data!! Number of time values != number of points" << std::endl;
        return -1;
    }

    // Check and remove old training data
    if(dmpInputData.times.size()>0)
        dmpInputData.times.clear();

    if(dmpInputData.points.size()>0)
        dmpInputData.points.clear();

    // Store new data
    size_t spaceDim = points[0].size(); // Assume position only is provided
    std::cout << "TrainDMP space dimension: " << spaceDim << std::endl;
    LWR_seed_control::DMPPoint tmpPoint;
    for(size_t j=0; j<spaceDim; ++j){
        tmpPoint.positions.push_back(0.0);
        tmpPoint.velocities.push_back(0.0);
    }

    for(size_t i=0; i<times.size(); ++i){
        dmpInputData.times.push_back(times[i]);
        for(size_t j=0; j<spaceDim; ++j){
            tmpPoint.positions[j]  = points[i][j];
            //tmpPoint.velocities[j] = points[i][j+spaceDim];
        }
        dmpInputData.points.push_back(tmpPoint);
    }

    return 0;
}


void GenerateDSTrajectoy::DMPTrajectory(vector<double> currFrame, double currentTime){
    // Compute next position
    bool posReach = computePositionDMP(currFrame, currentTime);

    // Compute next orientation
    bool oriReach = true;
    if(useOri_)
         oriReach = computeOrientationDMP(currFrame, currentTime);

    // Check goal reached
    if(posReach && oriReach)
        goalReached = true;
}


void GenerateDSTrajectoy::computeNextStateDMP( std::vector<LWR_seed_control::DMPData> dmpList,
                                               vector<double> currentState,
                                               vector<double> goal,
                                               vector<double> initialState,
                                               double tau,
                                               double currentTime,
                                               vector<double> &nextState )
{
    // Compute next state in each dimension
    size_t stateDim = size_t(currentState.size()/2);
    if(nextState.size()<currentState.size()){
        nextState.clear();
        for(size_t i=0; i<stateDim; i++){
            nextState.push_back(0.0);
        }
    }

    double f_eval;
    DMP_RosWrapper::FunctionApprox **f = new DMP_RosWrapper::FunctionApprox*[stateDim];
    for(int i=0; i<stateDim; i++)
        f[i] = new DMP_RosWrapper::LinearApprox(dmpList[i].f_domain, dmpList[i].f_targets);

    for(size_t i=0; i<stateDim; i++){
        double x = currentState[i];
        double v = currentState[i+stateDim];

        //Compute the phase and the log of the phase to assist with some numerical issues
        //Then, evaluate the function approximator at the log of the phase
        double s = DMP_RosWrapper::calcPhase(currentTime, tau);
        double log_s = currentTime/tau;
        if(log_s >= 1.0){
            f_eval = 0.0;
        }
        else{
            f_eval = f[i]->evalAt(log_s) * s;
        }

        //Update v dot and x dot based on DMP differential equations
        double v_dot = (dmpList[i].k_gain*((goal[i]-x) - (goal[i]-initialState[i])*s + f_eval) - dmpList[i].d_gain*v) / tau;
        double x_dot = v/tau;

        //Update state variables
        v += v_dot * sampleTime_;
        x += x_dot * sampleTime_;

        //Add current state to the plan
        nextState[i] = x;
        nextState[i+stateDim] = v;
    }

    //Clean up
    for(int i=0; i<stateDim; i++){
        delete f[i];
    }
    delete[] f;
}


void GenerateDSTrajectoy::checkSingularitiesRPY(std::vector<double> oldAngles, std::vector<double> &newAngles){
    // CHECK FOR SINGULARITIES +- 180deg
   /* double maxAngle = 170.0*3.1416/180.0;
    for(int i=0; i<3; ++i){
        if(newAngles[i]>maxAngle && oldAngles[i]<-0.0){
            newAngles[i] = -newAngles[i];
            if(newAngles[i]<-3.1416){
                newAngles[i] = -3.1416;
            }
        }
        else if(newAngles[i]<-maxAngle && oldAngles[i]>0.0){
            newAngles[i] = -newAngles[i];
            if(newAngles[i]>3.1416){
                newAngles[i] = 3.1416;
            }
        }
    }*/


    double maxAngle = 90.0*3.1416/180.0;
    for(int i=0; i<3; ++i){
        if(newAngles[i]>maxAngle && oldAngles[i]<-0.0){
            newAngles[i] = newAngles[i] - 2.0*3.1416;
        }
        else if(newAngles[i]<-maxAngle && oldAngles[i]>0.0){
            newAngles[i] = newAngles[i] + 2.0*3.1416;
        }
    }
}


void GenerateDSTrajectoy::checkSingularitiesRPY(Eigen::Vector3d oldAngles, std::vector<double> &newAngles){
    //bool found = false;
    double maxAngle = 90.0*3.1416/180.0;
    for(int i=0; i<3; ++i){
        if(newAngles[i]>maxAngle && oldAngles[i]<-0.0){
            newAngles[i] = newAngles[i] - 2.0*3.1416;
         //   found = true;
        }
        else if(newAngles[i]<-maxAngle && oldAngles[i]>0.0){
            newAngles[i] = newAngles[i] + 2.0*3.1416;
     //       found = true;
        }
    }
   // return found;
}


bool GenerateDSTrajectoy::computePositionDMP(vector<double> currFrame, double currTime){
    if(currTime<sampleTime_){
        dmpInitPos[0] = currFrame[3]; dmpInitPos[1] = currFrame[7]; dmpInitPos[2] = currFrame[11];
    }
    vector<double> nextState(6, 0.0);

    vector<double> goal(3, 0.0);
    goal[0] = goalPosition(0); goal[1] = goalPosition(1); goal[2] = goalPosition(2);
    //cout << goalPosition.transpose() << endl;

    vector<double> currentState(6);
    currentState[0] = currFrame[3]; currentState[1] = currFrame[7]; currentState[2] = currFrame[11];
    currentState[3] = currDmpVelocity[0]; currentState[4] = currDmpVelocity[1]; currentState[5] = currDmpVelocity[2];

    computeNextStateDMP( dmpPosition,
                         currentState,
                         goal,
                         dmpInitPos,
                         tauDmp,
                         currTime,
                         nextState );

    // Check convergence and store velocity
    double sum_ = 0.0;
    for(unsigned int i = 0; i < 3; ++i){
        currDmpVelocity[i] = nextState[i+3];
        sum_ += (nextState[i]-goalPosition[i])*(nextState[i]-goalPosition[i]);
    }

    SetPosOmogTranfVec(nextFrame_, nextState);

    if(sum_ <= endTrajCondition)
        return true;
    else
        return false;
}


bool GenerateDSTrajectoy::computeOrientationDMP(vector<double> currFrame, double currTime){
    vector<double> nextState(6, 0.0);

    if(currTime<sampleTime_){
        ExtractRotationMat(currFrame, currOrientation);
        currEulerAngles = currOrientation.eulerAngles(2, 1, 0);

        dmpInitOri[0] = currEulerAngles[0]; dmpInitOri[1] = currEulerAngles[1]; dmpInitOri[2] = currEulerAngles[2];
    }


    vector<double> goal(3, 0.0);
    goal[0] = goalEulerAngles(0); goal[1] = goalEulerAngles(1); goal[2] = goalEulerAngles(2);

    vector<double> currentState(6);
    currentState[0] = currEulerAngles[0]; currentState[1] = currEulerAngles[1]; currentState[2] = currEulerAngles[2];
    currentState[3] = currDmpAngVelocity[0]; currentState[4] = currDmpAngVelocity[1]; currentState[5] = currDmpAngVelocity[2];

    computeNextStateDMP( dmpOrientation,
                         currentState,
                         goal,
                         dmpInitOri,
                         tauDmp,
                         currTime,
                         nextState );

    // Check singularities
    checkSingularitiesRPY(currentState, nextState);

    // Check convergence and store velocity
    double sum_ = 0.0;
    for(unsigned int i = 0; i < 3; ++i){
        // Store next euler angles
        currEulerAngles[i] = nextState[i];
        // Store angular velocity
        currDmpAngVelocity[i] = nextState[i+3];
        // Check convergence
        sum_ += (nextState[i]-goalEulerAngles[i])*(nextState[i]-goalEulerAngles[i]);
    }

    SetOriOmogTranfVec(nextFrame_, nextState);

    if(sum_ <= endTrajOrientation)
        return true;
    else
        return false;
}


bool GenerateDSTrajectoy::computeOrientationLinDS(vector<double> currFrame){
    // Compute Euler angles
    //ExtractRotationMat(currFrame, currOrientation);
    //currEulerAngles = currOrientation.eulerAngles(2, 1, 0);

    vector<double> nextAngles(LEN_OF_CART_POS, 0.0);


    // Test
    /*double normOfVel = 0;
    for(unsigned int i = 0; i < LEN_OF_CART_POS; ++i){
        currAngVelocity_[i] = goalEulerAngles[i]-currEulerAngles[i];

        normOfVel += currAngVelocity_[i]*currAngVelocity_[i];
    }

    normOfVel = std::sqrt(normOfVel);
    for(unsigned int i = 0; i < LEN_OF_CART_POS; ++i){
        if(normOfVel>1e-8){
            currAngVelocity_[i] = (2.0*stiffness_[i])*currAngVelocity_[i]/normOfVel;
        }

        nextAngles[i] = currEulerAngles[i] + sampleTime_*currAngVelocity_[i];
    }*/
    //////////////////////

    for(unsigned int i = 0; i < LEN_OF_CART_ORI; ++i){
        nextAngles[i] = currEulerAngles[i] + sampleTime_*stiffness_[i]*(goalEulerAngles[i]-currEulerAngles[i]);
    }

    checkSingularitiesRPY(currEulerAngles, nextAngles);
    //    for(unsigned int i = 0; i < LEN_OF_CART_POS; ++i){
    //        currAngVelocity_[i] = (nextAngles[i] - currEulerAngles[i])/sampleTime_;
    //    }
    //}

    double sum_ = 0.0;
    for(unsigned int i = 0; i < LEN_OF_CART_ORI; ++i){
        sum_ += (nextAngles[i]-goalEulerAngles[i])*(nextAngles[i]-goalEulerAngles[i]);

        currEulerAngles[i] = nextAngles[i];
    }

     /*cout << "Goal: " << goalEulerAngles[0] << " " <<  goalEulerAngles[1] << " " << goalEulerAngles[2] << endl;
     cout << "Current: " << currEulerAngles[0] << " " << currEulerAngles[1] << " " << currEulerAngles[2] << endl;
     cout << "Commanded: " << nextAngles[0]*3.1416/180.0 << " " <<  nextAngles[1]*3.1416/180.0 << " " << nextAngles[2]*3.1416/180.0 << endl;*/

    SetOriOmogTranfVec(nextFrame_, nextAngles);
    //}

    if(sum_ <= endTrajOrientation)
        return true;
    else
        return false;
}


void GenerateDSTrajectoy::FirstOrderLSTraj(vector<double> currFrame)
{
    unsigned int i = 0, loopSize = LEN_OF_CART_POS;
    bool oriReach = true;
    if(useOri_){
        oriReach = computeOrientationLinDS(currFrame);
    }

    vector<double> currPosVec(LEN_OF_CART_POS, 0.0),
                   goalPosVec(LEN_OF_CART_POS, 0.0),
                   nextPosVec(LEN_OF_CART_POS, 0.0);

    double sum_ = 0.0;

    ExtractPositionVec(currFrame, currPosVec);
    ExtractPositionVec(goalFrame_, goalPosVec);

    // Test
    /*double normOfVel = 0;
    for(i = 0; i < loopSize; ++i){
        currLinVelocity_[i] = goalPosVec[i]-currPosVec[i];

        normOfVel += currLinVelocity_[i]*currLinVelocity_[i];
    }

    normOfVel = std::sqrt(normOfVel);
    for(i = 0; i < loopSize; ++i){
        if(normOfVel>0.05*0.05){
            currLinVelocity_[i] = stiffness_[i]*currLinVelocity_[i]/normOfVel;
        }

        nextPosVec[i] = currPosVec[i] + sampleTime_*currLinVelocity_[i];

        sum_ += (nextPosVec[i]-goalPosVec[i])*(nextPosVec[i]-goalPosVec[i]);
    }

    SetPosOmogTranfVec(nextFrame_, nextPosVec);

    if(sum_ <= endTrajCondition && oriReach){
        goalReached = true;
    }*/

    ////////////////////////////////////////

    for(i = 0; i < loopSize; ++i){
        currLinVelocity_[i] = stiffness_[i]*(goalPosVec[i]-currPosVec[i]);

        nextPosVec[i] = currPosVec[i] + sampleTime_*currLinVelocity_[i];

        sum_ += (nextPosVec[i]-goalPosVec[i])*(nextPosVec[i]-goalPosVec[i]);
    }

    SetPosOmogTranfVec(nextFrame_, nextPosVec);


    /*if((sum_<=0.04*0.04)){
        for(i = 0; i < loopSize; ++i){
            if(stiffness_[i] < 3.0){
                stiffness_[i] += 0.05;
            }
        }
    }*/


    if(sum_ <= endTrajCondition && oriReach){
        goalReached = true;
        for(i = 0; i < loopSize; ++i){
            stiffness_[i] = origStiff_[i];
        }
    }

    return;
}


void GenerateDSTrajectoy::FirstOrderDSNextPosition(vector<double> currFrame)
{
    unsigned int i = 0, loopSize = LEN_OF_CART_POS;

    vector<double> currPosVec(LEN_OF_CART_POS, 0.0),
                   goalPosVec(LEN_OF_CART_POS, 0.0),
                   nextPosVec(LEN_OF_CART_POS, 0.0);

    double sum_ = 0.0;

    ExtractPositionVec(currFrame, currPosVec);
    ExtractPositionVec(goalFrame_, goalPosVec);

    // Test
    for(i = 0; i < loopSize; ++i){
        nextPosVec[i] = currPosVec[i] + sampleTime_*currLinVelocity_[i];

        sum_ += (nextPosVec[i]-goalPosVec[i])*(nextPosVec[i]-goalPosVec[i]);
    }

    SetPosOmogTranfVec(nextFrame_, nextPosVec);

    if(sum_ <= endTrajCondition){
        goalReached = true;
    }

    return;
}


void GenerateDSTrajectoy::GetNextFrameFloatArray(float *NextCartFrame)
{
    unsigned int i = 0;
    for(i = 0; i < LEN_OF_CART_FRAME; ++i)
        NextCartFrame[i] = (float)nextFrame_[i];
}


void GenerateDSTrajectoy::ResetDSState( vector<double> goalFrame,
                                        vector<double> stiffness,
                                        vector<double> normDamping,
                                        double         sampleTime,
                                        bool useOrientation,
                                        unsigned int generationMode )
{
    SetGoal(goalFrame);
    SetStiffness(stiffness);
    SetDamping(normDamping);
    SetSampleTime(sampleTime);
    SetGoalReached(false);

    genMode_       = generationMode;
    useOri_        = useOrientation;

    // Store goal position
    goalPosition(0) = goalFrame_[3];
    goalPosition(1) = goalFrame_[7];
    goalPosition(2) = goalFrame_[11];

    // Store goal rotation matrix (eigen format)
    ExtractRotationMat(goalFrame_, goalOrientation);

    // Store goal Euler angles
    goalEulerAngles = goalOrientation.eulerAngles(0, 1, 2);
}


void GenerateDSTrajectoy::SetDamping(vector<double> normDamping)
{
    unsigned int i = 0;
    for(i = 0; i < LEN_OF_CART_FRAME; ++i)
        damping_[i] = 2.0*normDamping[i]*sqrt(stiffness_[i]);

    return;
}


void GenerateDSTrajectoy::ExtractPositionVec( vector<double> omogTransfVec,
                                              vector<double> &posVec )
{
    if(posVec.size()<LEN_OF_CART_POS)
    {
        posVec.clear();
        posVec.push_back(omogTransfVec[3]);
        posVec.push_back(omogTransfVec[7]);
        posVec.push_back(omogTransfVec[11]);
    }
    else
    {
        posVec[0] = omogTransfVec[3];
        posVec[1] = omogTransfVec[7];
        posVec[2] = omogTransfVec[11];
    }

    return;
}


void GenerateDSTrajectoy::eulerToRotMatrix( vector<double> zyxAngles,
                                            Eigen::Matrix<double, 3, 3> &rotMat )
{
    /*Eigen::AngleAxisd rollAngle(xyzAngles[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(xyzAngles[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(xyzAngles[2], Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    rotMat = q.matrix();*/

    rotMat = Eigen::AngleAxisd(zyxAngles[0], Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(zyxAngles[1], Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(zyxAngles[2], Eigen::Vector3d::UnitX());

}


void GenerateDSTrajectoy::SetOriOmogTranfVec( vector<double> &omogTransfVec,
                                              vector<double> zyxAngles )
{
    Eigen::Matrix<double, 3, 3> rotMat;
    eulerToRotMatrix(zyxAngles, rotMat);

    omogTransfVec[0] = rotMat(0,0); omogTransfVec[1] = rotMat(0,1); omogTransfVec[2] = rotMat(0,2);
    omogTransfVec[4] = rotMat(1,0); omogTransfVec[5] = rotMat(1,1); omogTransfVec[6] = rotMat(1,2);
    omogTransfVec[8] = rotMat(2,0); omogTransfVec[9] = rotMat(2,1); omogTransfVec[10] = rotMat(2,2);
}


void GenerateDSTrajectoy::ExtractRotationMat( vector<double> omogTransfVec,
                                              Eigen::Matrix<double, 3, 3> &rotMat )
{
    rotMat << omogTransfVec[0], omogTransfVec[1], omogTransfVec[2],
              omogTransfVec[4], omogTransfVec[5], omogTransfVec[6],
              omogTransfVec[8], omogTransfVec[9], omogTransfVec[10];
}


Eigen::Vector3d GenerateDSTrajectoy::rotationMatrixToRPY(Eigen::Matrix<double, 3, 3> Rotation){
    Eigen::Vector3d rpy;

    rpy(0) = atan2(Rotation(1,0),Rotation(0,0));
    rpy(1) = atan2(-Rotation(2,0), sqrt(Rotation(2,1)*Rotation(2,1) + Rotation(2,2)*Rotation(2,2)));
    rpy(2) = atan2(Rotation(2,1),Rotation(2,2));

    return rpy;
}


void GenerateDSTrajectoy::SetPosOmogTranfVec( vector<double> &omogTransfVec,
                                              vector<double> posVec )
{
    omogTransfVec[3]  = posVec[0];
    omogTransfVec[7]  = posVec[1];
    omogTransfVec[11] = posVec[2];
}
