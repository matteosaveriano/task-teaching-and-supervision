#include "LWR_seed_control/robotControl.h"

robotControl::robotControl(ros::NodeHandle node, double samplingTime):node_(node)
{
    pthread_mutex_init(&(this->executionMtx), NULL);

    executeBehaviour = true;

    currentPose.resize(CART_FRM_DIM);

    vector<double> goalFrame(CART_FRM_DIM, 0.0);
    vector<double> stiffness(LEN_OF_CART_POS, 0.7), normDamping(LEN_OF_CART_POS, 1.0);

    dsTraj = new GenerateDSTrajectoy( goalFrame,
                                      stiffness,
                                      normDamping,
                                      samplingTime,
                                      true,
                                      GenerateDSTrajectoy::LIN_DS );

    double dmpK = 70.0;
    for(int i=0; i<LEN_OF_CART_POS; ++i){
        dmpStiffness.push_back(dmpK);
        dmpDamping.push_back(2.0*sqrt(dmpK));
    }

    samplingTime_ = samplingTime;

    tfListener = new tf::TransformListener();

    tfPub = node_.advertise<tf::tfMessage>("/tf", 0);
    tfMsg.transforms.resize(1);
    tfMsg.transforms[0].child_frame_id = "/wsg50_end_link";
    tfMsg.transforms[0].header.frame_id = "/world";
    tfMsg.transforms[0].header.stamp = ros::Time::now();
    tfMsg.transforms[0].transform.translation.x = 0.0;
    tfMsg.transforms[0].transform.translation.y = 0.0;
    tfMsg.transforms[0].transform.translation.z = 0.0;
    tfMsg.transforms[0].transform.rotation.w = 1.0;
    tfMsg.transforms[0].transform.rotation.x = 0.0;
    tfMsg.transforms[0].transform.rotation.y = 0.0;
    tfMsg.transforms[0].transform.rotation.z = 0.0;
}


robotControl::~robotControl(){
    delete dsTraj;
    delete tfListener;
}


void robotControl::saveVectorMatrixToFile (string fileName, vector < vector <double> > outMat)
{
    std::ofstream out(fileName.data());
    if (!out)
    {
        cout << "No file found: " << fileName << endl;
        return;
    }
    int rows = (int)outMat.size();
    int cols = (int)outMat[0].size();
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            out << outMat[i][j] << "\t";
        }
        out << endl;
    }
    out.close();
    return;
}


int robotControl::goToGravityCompensation(){    
    bool goOn = true;
    while(goOn && ros::ok()){
        // No gravity compendation needed in simulation
        // Check if the robot has to be stopped
        pthread_mutex_lock(&executionMtx);
        goOn = executeBehaviour;
        pthread_mutex_unlock(&executionMtx);

        usleep(int(samplingTime_*1e6));
    }

    pthread_mutex_lock(&executionMtx);
    executeBehaviour = true;
    pthread_mutex_unlock(&executionMtx);

    return 0;
}


Eigen::Vector3d robotControl::rotationMatrixToRPY(Eigen::Matrix<double, 3, 3> Rotation){
    Eigen::Vector3d rpy;

    rpy(0) = atan2(Rotation(1,0),Rotation(0,0));
    rpy(1) = atan2(-Rotation(2,0), sqrt(Rotation(2,1)*Rotation(2,1) + Rotation(2,2)*Rotation(2,2)));
    rpy(2) = atan2(Rotation(2,1),Rotation(2,2));

    return rpy;
}


int robotControl::goToCartPose(vector<double> goalFrame){
    //if(startCartImpedanceCtrl()==-1){
    //    return -1;
    //}

    // Set goal pose
    dsTraj->SetGoal(goalFrame);
    // Set current pose
    bool poseFound = false;
    while(!poseFound && ros::ok()){
        poseFound = getGripperPoseTf(currentPose);
        usleep(int(samplingTime_*1e6));
    }
    if(!poseFound){
        return -1;
    }

    dsTraj->SetCurrEulerAngles(currentPose);

    std::cout << "Goal position: " << goalFrame[3] << "\t" << goalFrame[7] << "\t" << goalFrame[11] << std::endl;
    std::cout << "Curr position: " << currentPose[3] << "\t" << currentPose[7] << "\t" << currentPose[11] << std::endl;

    // Start motion
    bool goOn = true;
    stopExecution = false;
    while(!dsTraj->goalReached && goOn && ros::ok()){
        if(!stopExecution){
            dsTraj->update(currentPose);
            //cout << "Commanded first: " << commCartPose[3] << " " <<  commCartPose[7] << " " << commCartPose[11] << endl;
            dsTraj->GetNextFrameFloatArray(commCartPose);
            //cout << "Commanded after get: " << commCartPose[3] << " " <<  commCartPose[7] << " " << commCartPose[11] << endl;
        }

        //fri->GetMeasuredCartPose(commCartPose);
        for(int i=0; i<CART_FRM_DIM; ++i)
            currentPose[i] = commCartPose[i];

        // Publish current pose on tf
        publishGripperPoseTf(currentPose);

        // Check if the robot has to be stopped
        pthread_mutex_lock(&executionMtx);
        goOn = executeBehaviour;
        if(stopExecution){
            goOn = true;
        }
        pthread_mutex_unlock(&executionMtx);

        usleep(int(samplingTime_*1e6));
    }

    pthread_mutex_lock(&executionMtx);
    executeBehaviour = true;
    stopExecution    = false;
    pthread_mutex_unlock(&executionMtx);

    dsTraj->goalReached = false;

    return 0;
}


int robotControl::goToCartPoseDMP(vector<double> goalFrame, int dmpActionIndex){
    // Start Cartesian Impedance control
    //if(startCartImpedanceCtrl()==-1){
    //    return -1;
    //}

    // Set current pose
    bool poseFound = false;
    while(!poseFound && ros::ok()){
        poseFound = getGripperPoseTf(currentPose);
        usleep(int(samplingTime_*1e6));
    }
    if(!poseFound){
        return -1;
    }

    complexTraj[dmpActionIndex].SetCurrEulerAngles(currentPose);

    std::cout << "Inital position: " << currentPose[3] << " " << currentPose[7] << " " << currentPose[11] << std::endl;
    std::cout << "Initial angles: " << (180.0/3.14)*complexTraj[dmpActionIndex].getCurrEulerAngles().transpose() << std::endl;

    // Start motion
    bool goOn = true;
    stopExecution = false;
    double currentTime = 0.0;
    while(!complexTraj[dmpActionIndex].goalReached && goOn && ros::ok()){
        if(!stopExecution){
            complexTraj[dmpActionIndex].update(currentPose, currentTime);
            currentTime += samplingTime_;
            complexTraj[dmpActionIndex].GetNextFrameFloatArray(commCartPose);
        }

        for(int i=0; i<CART_FRM_DIM; ++i)
            currentPose[i] = commCartPose[i];

        // Publish current pose on tf
        publishGripperPoseTf(currentPose);

        // Check if the robot has to be stopped
        pthread_mutex_lock(&executionMtx);
        goOn = executeBehaviour;
        if(stopExecution){
            goOn = true;
        }
        pthread_mutex_unlock(&executionMtx);

        usleep(int(samplingTime_*1e6));
    }

    pthread_mutex_lock(&executionMtx);
    executeBehaviour = true;
    stopExecution = false;
    pthread_mutex_unlock(&executionMtx);

    complexTraj[dmpActionIndex].goalReached = false;

   // saveVectorMatrixToFile("anglesDMP", data);

    return 0;
}


void robotControl::publishGripperPoseTf(std::vector<double> pose){
    tfMsg.transforms[0].header.stamp = ros::Time::now();

    tfMsg.transforms[0].transform.translation.x = pose[3];
    tfMsg.transforms[0].transform.translation.y = pose[7];
    tfMsg.transforms[0].transform.translation.z = pose[11];

    tf::Matrix3x3 r_;
    r_[0][0] = pose[0]; r_[0][1] = pose[1]; r_[0][2] = pose[2];
    r_[1][0] = pose[4]; r_[1][1] = pose[5]; r_[1][2] = pose[6];
    r_[2][0] = pose[8]; r_[2][1] = pose[9]; r_[2][2] = pose[10];
    tf::Quaternion q_;
    r_.getRotation(q_);

    tfMsg.transforms[0].transform.rotation.w = q_.getW();
    tfMsg.transforms[0].transform.rotation.x = q_.getX();
    tfMsg.transforms[0].transform.rotation.y = q_.getY();
    tfMsg.transforms[0].transform.rotation.z = q_.getZ();

    // Publish tf message
    tfPub.publish(tfMsg);
}


bool robotControl::getGripperPoseTf(std::vector<double> &pose){
    tf::StampedTransform t_;

    try{
        tfListener->lookupTransform( "/world",
                                     "/wsg50_end_link",
                                      ros::Time(0),
                                      t_ );
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return false;
    }

    // Return current pose vector
    pose.resize(CART_FRM_DIM);
    // Position
    pose[3]  = t_.getOrigin().x();
    pose[7]  = t_.getOrigin().y();
    pose[11] = t_.getOrigin().z();
    // Rotation matrix
    tf::Matrix3x3 r_ = t_.getBasis();
    pose[0] = r_[0][0]; pose[1] = r_[0][1]; pose[2] = r_[0][2];
    pose[4] = r_[1][0]; pose[5] = r_[1][1]; pose[6] = r_[1][2];
    pose[8] = r_[2][0]; pose[9] = r_[2][1]; pose[10] = r_[2][2];

    return true;
}


/*void robotControl::initJointStateMsg(){
    seqCounter = 1;
    jointStateMsg.header.stamp = ros::Time::now();
    jointStateMsg.header.seq   = seqCounter;
    jointStateMsg.name.push_back("kimp_right_arm_0_joint");
    jointStateMsg.name.push_back("kimp_right_arm_1_joint");
    jointStateMsg.name.push_back("kimp_right_arm_2_joint");
    jointStateMsg.name.push_back("kimp_right_arm_3_joint");
    jointStateMsg.name.push_back("kimp_right_arm_4_joint");
    jointStateMsg.name.push_back("kimp_right_arm_5_joint");
    jointStateMsg.name.push_back("kimp_right_arm_6_joint");
    for (int i = 0; i < 7; i++){
        jointStateMsg.position.push_back(0.0);
    }

    jointStatePub = node_.advertise <sensor_msgs::JointState> ("/joint_states", 0);
    // The first time topics are not readed
    jointStatePub.publish(jointStateMsg);

    pthread_mutex_lock(&executionMtx);
    executeBehaviour = true;
    pthread_mutex_unlock(&executionMtx);
}*/
