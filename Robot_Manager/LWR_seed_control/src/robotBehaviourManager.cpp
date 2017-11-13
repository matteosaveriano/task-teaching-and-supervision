#include "LWR_seed_control/robotBehaviourManager.h"

robotBehaviourManager::robotBehaviourManager( ros::NodeHandle node,
                                              string objectListFile,
                                              float posThreshold,
                                              float velThreshold,
                                              std::string commandTopicName,
                                              string taskStateTopicName ):behNode(node)
{
    // Robot control object
    std::string packPath = ros::package::getPath("LWR_seed_control");
    robCtrl = new robotControl(behNode, 0.005);

    // Command interpreter object
    commInterp = new commandInterpreter(behNode, objectListFile, commandTopicName, taskStateTopicName);

    desiredPose.resize(LEN_OF_CART_POS);
    showPosition.resize(3);
    showPosition[0] = -0.7;
    showPosition[1] = 0.0;
    showPosition[2] = 0.35;

    // Leave the third angle unchanged
    showOrientation.resize(3);
    showOrientation[0] = 1.47;
    showOrientation[1] = 0.0;//-0.75;
    showOrientation[2] = 0.0;

    // tf listener
    tfListControl = new tf::TransformListener(ros::Duration(10));

    oldCmdGenBeh = oldCmdCtrlTh = -3;

    positionThresh = posThreshold;
    velocityThresh = velThreshold;

    // Thread to control (start/stop) tasks execution
    createThread(5, SCHED_FIFO, &taskControlThread);
    sleep(2);
    createThread(40, SCHED_FIFO, &generateRobotBehaviour);
    //sleep(2);
    //createThread(10, SCHED_FIFO, &actionSegmentation);
    ROS_INFO("robotBehaviourManager threads created \n");

    //ros::spinOnce(); //not sure if needed
}


robotBehaviourManager::~robotBehaviourManager(){
    // Stop thread
    termintateTaskThread_ = true;

    delete robCtrl;
    delete commInterp;
    delete tfListControl;
}


int robotBehaviourManager::executeRelativeReaching(int objectIndex, int actionIndex){
    std::vector<double> desPose(CART_FRM_DIM);
    int res_;

    pthread_mutex_lock(&commInterp->callbackMtx);
    // Find desired pose
    res_ = findRobotObjectPoseNew(commInterp->objects->objectList[objectIndex], desPose, actionIndex);
    if(res_ == -1) return TASK_FAILED;
    pthread_mutex_unlock(&commInterp->callbackMtx);

    // Go to desired pose
    res_ = robCtrl->goToCartPose(desPose);
    if(res_ == -1) return TASK_FAILED;

    return TASK_FINISHED;
}


void robotBehaviourManager::saveLearnedTask(std::string pathName){
    // Save learned DMP
    cout << "Number of complex actions: " << (int)robCtrl->complexTraj.size() << endl;
    for(int i=0; i<(int)robCtrl->complexTraj.size(); ++i){
        std::string fileName = pathName + "action" + commInterp->numToString(i);
        robCtrl->complexTraj[i].saveDMPparamsToFile(fileName);
    }

    // Store number of actions
    ofstream outPos((pathName+"actionNumber.txt").data());
    if (!outPos){
        std::cout << "No file found: " << pathName+"actionNumber.txt" << std::endl;
        return;
    }
    outPos << robCtrl->complexTraj.size() << endl;
    outPos.close();

    // Store action index map
    std::ofstream ofs((pathName+"actionIndexMap.txt").data());
    boost::archive::text_oarchive oa(ofs);
    oa << robCtrl->actionIndexMap;
    ofs.close();
}


bool robotBehaviourManager::loadLearnedTask(std::string pathName){
    // Read number of actions to load
    ifstream in((pathName+"actionNumber.txt").data());
    if (!in){
        cout << "No file found: " << pathName+"actionNumber.txt" << endl;
        return false;
    }
    int actionNum;
    in >> actionNum;
    in.close();

    // Load all actions
    std::vector<double> goalFrame(CART_FRM_DIM, 0.0);
    for(int i=0; i<actionNum; ++i){
        GenerateDSTrajectoy emptyDmp( goalFrame,
                                      robCtrl->dmpStiffness,
                                      robCtrl->dmpDamping,
                                      robCtrl->getSamplingTime(),
                                      true,
                                      GenerateDSTrajectoy::DMP );

        std::string fileName = pathName + "action" + commInterp->numToString(i);
        if(emptyDmp.loadDMPparamsFromFile(fileName, 3)){
            robCtrl->complexTraj.push_back(emptyDmp);
        }
        else{
            return false;
        }
    }

    // Load action index map
    std::ifstream ifs((pathName+"actionIndexMap.txt").data());
    boost::archive::text_iarchive ia(ifs);
    ia >> robCtrl->actionIndexMap;
    ifs.close();

    return true;
}


int robotBehaviourManager::executeAbsoluteReaching(int objectIndex, int actionIndex){
    std::vector<double> desPose(CART_FRM_DIM);

    // Get desired pose
    pthread_mutex_lock(&commInterp->callbackMtx);
    tf::Transform relPose = stdVectorsToTfTranform(commInterp->objects->objectList[objectIndex].actions_[actionIndex].positions_[0],
                                                   commInterp->objects->objectList[objectIndex].actions_[actionIndex].orientations_[0] );
    pthread_mutex_unlock(&commInterp->callbackMtx);

    // Position
    desPose[3]  = relPose.getOrigin()[0];
    desPose[7]  = relPose.getOrigin()[1];
    desPose[11] = relPose.getOrigin()[2];
    // Rotation
    desPose[0] = relPose.getBasis()[0][0]; desPose[1] = relPose.getBasis()[0][1]; desPose[2]  = relPose.getBasis()[0][2];
    desPose[4] = relPose.getBasis()[1][0]; desPose[5] = relPose.getBasis()[1][1]; desPose[6]  = relPose.getBasis()[1][2];
    desPose[8] = relPose.getBasis()[2][0]; desPose[9] = relPose.getBasis()[2][1]; desPose[10] = relPose.getBasis()[2][2];

    // Go to desired pose
    int res_ = robCtrl->goToCartPose(desPose);
    if(res_ == -1) return TASK_FAILED;

    return TASK_FINISHED;
}


int robotBehaviourManager::executeDeltaReaching(int objectIndex, int actionIndex){
    // Get Current Pose
    std::vector<double> desPose(CART_FRM_DIM);
    bool poseFound = false;
    while(!poseFound && ros::ok()){
        poseFound = robCtrl->getGripperPoseTf(desPose);
        usleep(int(robCtrl->getSamplingTime()*1e6));
    }
    if(!poseFound){
        return -1;
    }

    // Add position displacement (orientation neglected!!!)
    pthread_mutex_lock(&commInterp->callbackMtx);
    desPose[3] += commInterp->objects->objectList[objectIndex].actions_[actionIndex].positions_[0][0];
    desPose[7] += commInterp->objects->objectList[objectIndex].actions_[actionIndex].positions_[0][1];
    desPose[11] += commInterp->objects->objectList[objectIndex].actions_[actionIndex].positions_[0][2];
    pthread_mutex_unlock(&commInterp->callbackMtx);

    // Go to desired pose
    int res_ = robCtrl->goToCartPose(desPose);
    if(res_ == -1) return TASK_FAILED;

    return TASK_FINISHED;
}


int robotBehaviourManager::executeComplexAction(int objectIndex, int actionIndex){
    // Find complex action index
    int compActInd = robCtrl->actionIndexMap[commInterp->objects->objectList[objectIndex].actions_[actionIndex].label_];

    std::cout << "executeComplexAction debug" << std::endl;
    std::cout << "compActInd: " << compActInd << std::endl;
    std::cout << "complexTraj size: " << robCtrl->complexTraj.size() << std::endl;

    // Store learned action
    robCtrl->goToCartPoseDMP(robCtrl->complexTraj[compActInd].goalFrame_, compActInd);

    return TASK_FINISHED;
}


int robotBehaviourManager::executeTaskRoutine(int objectIndex, int actionIndex){
    std::cout << objectIndex << " " << actionIndex << endl;
    std::string actionFrame;
    pthread_mutex_lock(&commInterp->callbackMtx);
    std::cout << commInterp->objects->objectList.size() << " " << commInterp->objects->objectList[objectIndex].actions_.size() << endl;
    actionFrame = commInterp->objects->objectList[objectIndex].actions_[actionIndex].frame_;
    pthread_mutex_unlock(&commInterp->callbackMtx);

    int res_ = TASK_FAILED;
    if(actionFrame == "world"){
        res_ = executeAbsoluteReaching(objectIndex, actionIndex);
    }
    else if(actionFrame == "worldDelta"){
        res_ = executeDeltaReaching(objectIndex, actionIndex);
    }
    else if(actionFrame == "worldComplex"){
        res_ = executeComplexAction(objectIndex, actionIndex);
    }
    else{ // relative action (take)
        res_ = executeRelativeReaching(objectIndex, actionIndex);
    }

    return res_;
}


void *robotBehaviourManager::generateRobotBehaviour(void* objectPointer){
    robotBehaviourManager* objPtr = (robotBehaviourManager*)objectPointer;

    pthread_mutex_lock(&(objPtr->mutexVar_));
    objPtr->threadCreated_	=	true;
    pthread_mutex_unlock(&(objPtr->mutexVar_));

    pthread_cond_signal(&(objPtr->condVar_));

    pthread_mutex_lock(&(objPtr->mutexVar_));
    while (!objPtr->taskThreadReady_){
        pthread_cond_wait(&(objPtr->condVar_), &(objPtr->mutexVar_));
    }
    pthread_mutex_unlock(&(objPtr->mutexVar_));

    while(ros::ok() && !objPtr->termintateTaskThread_){
        pthread_mutex_lock(&objPtr->commInterp->callbackMtx);
        int currComm      = objPtr->commInterp->robotCommand;
        int objectToUseId = objPtr->commInterp->objectToUseId;
        int objectToUseIndex = objPtr->commInterp->objectToUseIndex;
        int actionIndex   = objPtr->commInterp->actionToPerformIndex;
        int dim_          = objPtr->commInterp->objects->objectList.size();
        int lastObjectId  = objPtr->commInterp->objects->objectList[objPtr->commInterp->learnObjIndx].id;
        pthread_mutex_unlock(&objPtr->commInterp->callbackMtx);

        objPtr->taskStatus = TASK_EXECUTION;

        int oldCmd = objPtr->oldCmdGenBeh;

        if(oldCmd != currComm){
            switch(currComm){
            // HERE I DO EVERYTHING!!
            case(commandInterpreter::CMD_EXECUTE):
                /*pthread_mutex_lock(&(objPtr->commInterp->callbackMtx));
                objPtr->commInterp->taskExecuted = false;
                pthread_mutex_unlock(&(objPtr->commInterp->callbackMtx));*/

                ROS_INFO("Received command CMD_EXECUTE\n");
                fflush(stdout);

                objPtr->taskStatus = objPtr->executeTaskRoutine(objectToUseIndex, actionIndex);
                break;

            case(commandInterpreter::CMD_TEACH):
                ROS_INFO("Received command CMD_TEACH\n");
                fflush(stdout);
                pthread_mutex_lock(&objPtr->commInterp->callbackMtx);
                objPtr->commInterp->isTeachMode = true;
                pthread_mutex_unlock(&objPtr->commInterp->callbackMtx);

                objPtr->taskStatus = TASK_FINISHED;

                objPtr->robCtrl->goToGravityCompensation();

                ROS_INFO("Teaching finished\n");

                pthread_mutex_lock(&objPtr->commInterp->callbackMtx);
                objPtr->commInterp->isTeachMode = false;
                pthread_mutex_unlock(&objPtr->commInterp->callbackMtx);
                break;

            /*case(commandInterpreter::CMD_REPEAT):
                ROS_INFO("Received command CMD_REPEAT\n");
                fflush(stdout);
                objPtr->taskStatus = objPtr->repeatTaskRoutine(6);
                break;
            case(commandInterpreter::CMD_PLACE):
                ROS_INFO("Received command CMD_PLACE object %i\n", objectToUseId);
                fflush(stdout);
                objPtr->taskStatus = objPtr->placeObjectRoutine(objectToUseId);
                break;

            case(commandInterpreter::CMD_TAKE):
                ROS_INFO("Received command CMD_TAKE object %i\n", objectToUseId);
                fflush(stdout);
                objPtr->taskStatus = objPtr->takeObjectRoutine(objectToUseId);
                break;

            case(commandInterpreter::CMD_SHOW):
                ROS_INFO("Received command CMD_SHOW\n");
                objPtr->taskStatus = objPtr->showObjectRoutine();
                fflush(stdout);
                break;*/

            case(commandInterpreter::CMD_TERMIN):
                // Stop thread
                pthread_mutex_lock(&(objPtr->mutexVar_));
                objPtr->termintateTaskThread_ = true;
                pthread_mutex_unlock(&(objPtr->mutexVar_));

                ROS_INFO("Received command CMD_TERMIN\n");
                fflush(stdout);

                break;

            default:
                ROS_INFO("Received command DEFAULT\n");
                fflush(stdout);
                objPtr->taskStatus = TASK_FINISHED;
                break;
            }
            //ROS_INFO("Fine if\n");
        }

        //ROS_INFO("Thread %i says: old %i - new %i", 1, oldCmd, currComm);

        objPtr->oldCmdGenBeh = currComm;

        usleep(1000000); // 1s
    }

   // pthread_mutex_lock(&(objPtr->mutexVar_));
    objPtr->termintateTaskThread_ = true;
   // pthread_mutex_unlock(&(objPtr->mutexVar_));   

    ROS_INFO("generateRobotBehaviour terminated");

    pthread_exit(NULL);
    return(NULL);
}


// OLD version
/*void *robotBehaviourManager::generateRobotBehaviour(void* objectPointer){
    robotBehaviourManager* objPtr = (robotBehaviourManager*)objectPointer;

    pthread_mutex_lock(&(objPtr->mutexVar_));
    objPtr->threadCreated_	=	true;
    pthread_mutex_unlock(&(objPtr->mutexVar_));

    pthread_cond_signal(&(objPtr->condVar_));

    pthread_mutex_lock(&(objPtr->mutexVar_));
    while (!objPtr->taskThreadReady_){
        pthread_cond_wait(&(objPtr->condVar_), &(objPtr->mutexVar_));
    }
    pthread_mutex_unlock(&(objPtr->mutexVar_));

    while(ros::ok() && !objPtr->termintateTaskThread_){
        pthread_mutex_lock(&objPtr->commInterp->callbackMtx);
        int currComm      = objPtr->commInterp->robotCommand;
        int objectToUseId = objPtr->commInterp->objectToUseId;
        int dim_          = objPtr->commInterp->objects->objectList.size();
        int lastObjectId  = objPtr->commInterp->objects->objectList[objPtr->commInterp->learnObjIndx].id;
        pthread_mutex_unlock(&objPtr->commInterp->callbackMtx);

        objPtr->taskStatus = TASK_EXECUTION;

        int oldCmd = objPtr->oldCmdGenBeh;

        if(oldCmd != currComm){
            pthread_mutex_lock(&(objPtr->mutexVar_));
            objPtr->isInTeachMode = false;
            pthread_mutex_unlock(&(objPtr->mutexVar_));
            switch(currComm){
            case(commandInterpreter::CMD_EXECUTE):
                ROS_INFO("Received command CMD_EXECUTE\n");
                fflush(stdout);
                objPtr->taskStatus = TASK_FINISHED;
                break;

            case(commandInterpreter::CMD_TEACH):
                ROS_INFO("Received command CMD_TEACH\n");
                fflush(stdout);
                pthread_mutex_lock(&(objPtr->mutexVar_));
                objPtr->isInTeachMode = false;
                pthread_mutex_unlock(&(objPtr->mutexVar_));

                objPtr->taskStatus = TASK_FINISHED;

                objPtr->robCtrl->goToGravityCompensation();
                break;

            case(commandInterpreter::CMD_PLACE):
                ROS_INFO("Received command CMD_PLACE object %i\n", objectToUseId);
                fflush(stdout);
                objPtr->taskStatus = objPtr->placeObjectRoutine(objectToUseId);
                break;

            case(commandInterpreter::CMD_TAKE):
                ROS_INFO("Received command CMD_TAKE object %i\n", objectToUseId);
                fflush(stdout);
                objPtr->taskStatus = objPtr->takeObjectRoutine(objectToUseId);
                break;

            case(commandInterpreter::CMD_SHOW):
                ROS_INFO("Received command CMD_SHOW\n");
                objPtr->taskStatus = objPtr->showObjectRoutine();
                fflush(stdout);
                break;

            case(commandInterpreter::CMD_TERMIN):
                // Stop thread
                // Set zero stiffness to avoid jumps
                float cartStiffnessValues[6];
                for (int i=0; i<FRI_CART_VEC; ++i){
                    cartStiffnessValues[i]   = (float)MIN_CART_LIN_STIF;
                }
                objPtr->robCtrl->fri->SetCommandedCartStiffness(cartStiffnessValues);

                pthread_mutex_lock(&(objPtr->mutexVar_));
                objPtr->termintateTaskThread_ = true;
                pthread_mutex_unlock(&(objPtr->mutexVar_));

                ROS_INFO("Received command CMD_TERMIN\n");
                fflush(stdout);

                break;

            default:
                ROS_INFO("Received command DEFAULT\n");
                fflush(stdout);
                objPtr->taskStatus = TASK_FINISHED;
                break;
            }
            //ROS_INFO("Fine if\n");
        }

        //ROS_INFO("Thread %i says: old %i - new %i", 1, oldCmd, currComm);

        objPtr->oldCmdGenBeh = currComm;

        usleep(1000000); // 1s
    }

   // pthread_mutex_lock(&(objPtr->mutexVar_));
    objPtr->termintateTaskThread_ = true;
   // pthread_mutex_unlock(&(objPtr->mutexVar_));

    int resultValue	= objPtr->robCtrl->fri->StopRobot();
    if (resultValue != EOK){
        ROS_ERROR("An error occurred during stopping the robot!\n");
    }

    ROS_INFO("generateRobotBehaviour terminated");

    pthread_exit(NULL);
    return(NULL);
}*/



void* robotBehaviourManager::taskControlThread(void* objectPointer){
    robotBehaviourManager* objPtr = (robotBehaviourManager*)objectPointer;

    pthread_mutex_lock(&(objPtr->mutexVar_));
    objPtr->threadCreated_	=	true;
    pthread_mutex_unlock(&(objPtr->mutexVar_));

    pthread_cond_signal(&(objPtr->condVar_));

    pthread_mutex_lock(&(objPtr->mutexVar_));
    while (!objPtr->taskThreadReady_){
        pthread_cond_wait(&(objPtr->condVar_), &(objPtr->mutexVar_));
    }
    pthread_mutex_unlock(&(objPtr->mutexVar_));

    // Read current command
    while(!objPtr->termintateTaskThread_ && ros::ok()){
        pthread_mutex_lock(&(objPtr->commInterp->callbackMtx));
        int currCmd = objPtr->commInterp->robotCommand;
        pthread_mutex_unlock(&(objPtr->commInterp->callbackMtx));

        int oldCmd = objPtr->oldCmdCtrlTh;

        if(oldCmd != currCmd){
            if(currCmd == commandInterpreter::CMD_STOP){ // Stop execution
                pthread_mutex_lock(&(objPtr->robCtrl->executionMtx));
                //objPtr->robCtrl->executeBehaviour = false;
                objPtr->robCtrl->stopExecution = true;
                pthread_mutex_unlock(&(objPtr->robCtrl->executionMtx));

                ROS_INFO("Received command CMD_STOP\n");
                fflush(stdout);


                //objPtr->taskStatus = TASK_STOPPED;
            }
            else if(currCmd == commandInterpreter::CMD_START){ // Start execution
                pthread_mutex_lock(&(objPtr->robCtrl->executionMtx));
                //objPtr->robCtrl->executeBehaviour = true;
                objPtr->robCtrl->stopExecution = false;
                pthread_mutex_unlock(&(objPtr->robCtrl->executionMtx));

                ROS_INFO("Received command CMD_START\n");
                fflush(stdout);

                //if(objPtr->taskStatus == TASK_STOPPED){
                //    objPtr->taskStatus = TASK_RESTART;
                //}
            }
             else if(currCmd == commandInterpreter::CMD_DONE){ // Teach finished
                ROS_INFO("Received command CMD_DONE\n");
                fflush(stdout);

                pthread_mutex_lock(&(objPtr->robCtrl->executionMtx));
                objPtr->robCtrl->executeBehaviour = false;
                pthread_mutex_unlock(&(objPtr->robCtrl->executionMtx));

                objPtr->taskStatus = TASK_EXECUTION;

                // Open gripper in case
                std_msgs::Int32 cmd_;
                cmd_.data = 2;
                objPtr->commInterp->gripperControl(cmd_);

                // Train complex actions
                // Find complex actions
                size_t objNum = objPtr->commInterp->objects->objectList.size();
                for(size_t i=0; i<objNum; ++i){
                    size_t actionNum = objPtr->commInterp->objects->objectList[i].actions_.size();
                    for(size_t j=0; j<actionNum; ++j){
                        if(objPtr->commInterp->objects->objectList[i].actions_[j].frame_=="worldComplex"){
                            // Create DMP onbject
                            std::vector<double> goalFrame(CART_FRM_DIM, 0.0);
                            GenerateDSTrajectoy emptyDmp( goalFrame,
                                                          objPtr->robCtrl->dmpStiffness,
                                                          objPtr->robCtrl->dmpDamping,
                                                          objPtr->robCtrl->getSamplingTime(),
                                                          true,
                                                          GenerateDSTrajectoy::DMP );
                            //void trainDMP(vector<vector<double> > inputData, vector<double> times, bool position);
                            // Set tau
                            int dim_ = (int)objPtr->commInterp->objects->objectList[i].actions_[j].positions_.size();
                            int dimOri = (int)objPtr->commInterp->objects->objectList[i].actions_[j].orientations_.size();
                            if(dim_ == dimOri){
                                // Train DMP position
                                // downsampling
                                objPtr->splineDownSampling(objPtr->commInterp->objects->objectList[i].actions_[j].positions_, 0.3);
                                std::cout << "Resampled position size: " <<(int)objPtr->commInterp->objects->objectList[i].actions_[j].positions_.size() << std::endl;

                                //objPtr->robCtrl->saveVectorMatrixToFile("anglesPreSpline", objPtr->commInterp->objects->objectList[i].actions_[j].orientations_);
                                objPtr->splineDownSampling(objPtr->commInterp->objects->objectList[i].actions_[j].orientations_, 0.3);
                                std::cout << "Resampled orientation size: " <<(int)objPtr->commInterp->objects->objectList[i].actions_[j].orientations_.size() << std::endl;
                                //objPtr->robCtrl->saveVectorMatrixToFile("anglesPostSpline", objPtr->commInterp->objects->objectList[i].actions_[j].orientations_);


                                dim_ = (int)objPtr->commInterp->objects->objectList[i].actions_[j].positions_.size();
                                double kukaTime = objPtr->robCtrl->getSamplingTime();
                                emptyDmp.setTauDmp(double(dim_*kukaTime*1.5));
                                 std::cout << "Tau: " << double(dim_*kukaTime) << std::endl;
                                // Create time vector
                                std::vector<double> times(dim_, 0.0);
                                for(int t_=1; t_<dim_; ++t_){
                                    times[t_] = times[t_-1] + kukaTime;
                                }

                                std::cout << "Resampled time size: " << times.size() << std::endl;

                                emptyDmp.trainDMP( objPtr->commInterp->objects->objectList[i].actions_[j].positions_,
                                                    times,
                                                    true );
                                // Train DMP orientation
                                emptyDmp.trainDMP( objPtr->commInterp->objects->objectList[i].actions_[j].orientations_,
                                                    times,
                                                    false );
                                // Store learned action
                                objPtr->robCtrl->complexTraj.push_back(emptyDmp);
                                // Set Goal
                                objPtr->robCtrl->complexTraj[(int)objPtr->robCtrl->complexTraj.size()-1].setGoalFrameDMP();

                                // Update action map
                                objPtr->robCtrl->actionIndexMap[objPtr->commInterp->objects->objectList[i].actions_[j].label_] =
                                                                        (int)objPtr->robCtrl->complexTraj.size()-1;
                            }
                            else{
                                ROS_ERROR("Object: %d, Action: %d position/orientation sizes do not match!", i, j);
                            }
                        }
                    }
                }

		      //  std::string cc;

		       // std::cout << "Teaching finished. Press a key to continue...\n";
		        //std::cin >> cc;
                //objPtr->robCtrl->emptyDmp->trainDMP();
                //commInterp->objects->objectList[objectIndex].actions_[actionIndex].positions_[0],
                //                                                   commInterp->objects->objectList[objectIndex].actions_[actionIndex].orientations_[0]

            }
            else if(currCmd == commandInterpreter::CMD_TERMIN){
                // Stop current task
                pthread_mutex_lock(&(objPtr->robCtrl->executionMtx));
                objPtr->robCtrl->executeBehaviour = false;
                pthread_mutex_unlock(&(objPtr->robCtrl->executionMtx));
                // Terminate threads
                pthread_mutex_lock(&(objPtr->mutexVar_));
                objPtr->termintateTaskThread_ = true;
                pthread_mutex_unlock(&(objPtr->mutexVar_));

                ROS_INFO("Received command CMD_TERMIN\n");
                fflush(stdout);

                objPtr->taskStatus = TASK_STOPPED;
            }
        }

        objPtr->oldCmdCtrlTh = currCmd;

        // Publish task status
        if(objPtr->taskStatus == TASK_FINISHED){
          //  std::cout << objPtr->taskStatus << std::endl;

            pthread_mutex_lock(&(objPtr->commInterp->callbackMtx));
            objPtr->commInterp->robotCommand = -1;
            objPtr->commInterp->taskExecuted = true;
            pthread_mutex_unlock(&(objPtr->commInterp->callbackMtx));
        }

        usleep(1000000);
    }

   // pthread_mutex_lock(&(objPtr->mutexVar_));
    objPtr->termintateTaskThread_ = true;
   // pthread_mutex_unlock(&(objPtr->mutexVar_));

    ROS_INFO("taskControlThread terminated");

    pthread_exit(NULL);
    return (NULL);
}


void robotBehaviourManager::splineDownSampling(vector< vector <double> > &desPos, double samplingTime){
    int spaceDim = (int)desPos[0].size();
    tk::spline *splineTraj = new tk::spline[spaceDim];

    // Generate time vectors
    std::vector<double> timeKinect, cartVal;
    int trajSamples = (int)desPos.size();
    timeKinect.resize(trajSamples+1);
    cartVal.resize(trajSamples+1);

    int i, j = 0;
    double t_ = 0.0;
    for(i=0; i<trajSamples+1; ++i) {
        timeKinect[i] = t_;
        t_ += samplingTime;
    }

    for(j=0; j<spaceDim; ++j) {
        for(i=0; i<trajSamples; ++i) {
            cartVal[i] = desPos[i][j];
        }
        cartVal[trajSamples] = cartVal[trajSamples-1];
        splineTraj[j].set_points(timeKinect, cartVal);    // currently it is required that X is already sorted
    }


    t_ = 0.0;
    double dt_ = robCtrl->getSamplingTime();
    int robTrajSamples = (trajSamples+1)*(int)std::floor(samplingTime/dt_);

    // Initial values
    desPos.clear();
    desPos.push_back(vector<double>());
    for(j=0; j<spaceDim; ++j) {
        desPos[0].push_back(splineTraj[j](t_));
    }

    // Loop
    for(i=1; i<robTrajSamples; ++i) {
        desPos.push_back(vector<double>());

        t_ += dt_;
        for(j=0; j<spaceDim; ++j) {
            desPos[i].push_back(splineTraj[j](t_));
        }
    }
}


int robotBehaviourManager::findRobotObjectPoseNew( objectsList::objectsDescription object,
                                                   std::vector<double> &desPose,
                                                   int actionIndex ){
    // Set right frame
    std::string   objId;
    tf::Transform relPose;

    objId = commInterp->numToString(object.id);
    // Relative pose in tf format
    relPose = stdVectorsToTfTranform( object.actions_[actionIndex].positions_[0],
                                      object.actions_[actionIndex].orientations_[0] );
    // Read qr code pose
    int found = commInterp->waitTfTransform("world", "ar_marker_" + objId, qrPose);
    if(!found){
        ROS_ERROR("Impossible to find a transformation from %s to %s \n", ("ar_marker_" + objId).c_str(), "world");
        return -1;
    }

    //cout << "relPose: " << relPose.getOrigin()[0] << " " << relPose.getOrigin()[1] << " " << relPose.getOrigin()[2] << endl;
    //cout << "takeOrientation: " << object.takeOrientation[0]*180.0/3.14 << " " <<  object.takeOrientation[1]*180.0/3.14 << " " << object.takeOrientation[2]*180.0/3.14 << endl;
    // Find absolute pose
    tf::Transform testPose(qrPose.getBasis(), qrPose.getOrigin());
    relPose.mult(testPose, relPose);

    // Convert to std::vector
    // Position
    desPose[3]  = relPose.getOrigin()[0];
    desPose[7]  = relPose.getOrigin()[1];
    desPose[11] = relPose.getOrigin()[2];

    // Orientation
    desPose[0] = relPose.getBasis()[0][0]; desPose[1] = relPose.getBasis()[0][1]; desPose[2]  = relPose.getBasis()[0][2];
    desPose[4] = relPose.getBasis()[1][0]; desPose[5] = relPose.getBasis()[1][1]; desPose[6]  = relPose.getBasis()[1][2];
    desPose[8] = relPose.getBasis()[2][0]; desPose[9] = relPose.getBasis()[2][1]; desPose[10] = relPose.getBasis()[2][2];

    return 0;
}


int robotBehaviourManager::findRobotObjectPose( objectsList::objectsDescription object,
                                                std::vector<double> &desPose,
                                                bool releaseObject ){
    // Set right frame
    std::string   objId;
    tf::Transform relPose;
    if(releaseObject){
        objId = commInterp->numToString(object.trayId);
        // Relative pose in tf format
        relPose = stdVectorsToTfTranform( object.placePosition,
                                          object.placeOrientation );
    }
    else{
        objId = commInterp->numToString(object.id);
        // Relative pose in tf format
        relPose = stdVectorsToTfTranform( object.takePosition,
                                          object.takeOrientation );
    }
    // Read qr code pose
    int found = commInterp->waitTfTransform("world", "ar_marker_" + objId, qrPose);
    if(!found){
        ROS_ERROR("Impossible to find a transformation from %s to %s \n", ("ar_marker_" + objId).c_str(), "world");
        return -1;
    }

    //cout << "relPose: " << relPose.getOrigin()[0] << " " << relPose.getOrigin()[1] << " " << relPose.getOrigin()[2] << endl;
    //cout << "takeOrientation: " << object.takeOrientation[0]*180.0/3.14 << " " <<  object.takeOrientation[1]*180.0/3.14 << " " << object.takeOrientation[2]*180.0/3.14 << endl;
    // Find absolute pose
    tf::Transform testPose(qrPose.getBasis(), qrPose.getOrigin());
    relPose.mult(testPose, relPose);

    /*cout << "qrPose: " << endl;
    cout << qrPose.getBasis()[0][0] << " " << qrPose.getBasis()[0][1] << " " << qrPose.getBasis()[0][2] << " " << qrPose.getOrigin().x() << endl;
    cout << qrPose.getBasis()[1][0] << " " << qrPose.getBasis()[1][1] << " " << qrPose.getBasis()[1][2] << " " << qrPose.getOrigin().y() << endl;
    cout << qrPose.getBasis()[2][0] << " " << qrPose.getBasis()[2][1] << " " << qrPose.getBasis()[2][2] << " " << qrPose.getOrigin().z() << endl;

    cout << "relPose: " << endl;
    cout << relPose.getBasis()[0][0] << " " << relPose.getBasis()[0][1] << " " << relPose.getBasis()[0][2] << " " << relPose.getOrigin().x() << endl;
    cout << relPose.getBasis()[1][0] << " " << relPose.getBasis()[1][1] << " " << relPose.getBasis()[1][2] << " " << relPose.getOrigin().y() << endl;
    cout << relPose.getBasis()[2][0] << " " << relPose.getBasis()[2][1] << " " << relPose.getBasis()[2][2] << " " << relPose.getOrigin().z() << endl;*/

    // Convert to std::vector
    // Position
    desPose[3]  = relPose.getOrigin()[0];
    desPose[7]  = relPose.getOrigin()[1];
    desPose[11] = relPose.getOrigin()[2];

   /* cout << "desPose: " << desPose[3] << " " << desPose[7] << " " << desPose[11] << endl;
    cout << "desPose: " << absPose.getBasis()[0][0] << " " << absPose.getBasis()[0][1] << " " << absPose.getBasis()[0][2] << endl;
    cout << "desPose: " << absPose.getBasis()[1][0] << " " << absPose.getBasis()[1][1] << " " << absPose.getBasis()[1][2] << endl;
    cout << "desPose: " << absPose.getBasis()[2][0] << " " << absPose.getBasis()[2][1] << " " << absPose.getBasis()[2][2] << endl;*/

    // Orientation
    desPose[0] = relPose.getBasis()[0][0]; desPose[1] = relPose.getBasis()[0][1]; desPose[2]  = relPose.getBasis()[0][2];
    desPose[4] = relPose.getBasis()[1][0]; desPose[5] = relPose.getBasis()[1][1]; desPose[6]  = relPose.getBasis()[1][2];
    desPose[8] = relPose.getBasis()[2][0]; desPose[9] = relPose.getBasis()[2][1]; desPose[10] = relPose.getBasis()[2][2];

    return 0;
}


tf::Transform robotBehaviourManager::stdVectorsToTfTranform( std::vector<double> position,
                                                             std::vector<double> orientationYPR ){
    // Position to tf::Vector3
    tf::Vector3 pos_(position[0], position[1], position[2]);
    // Orientation to tf::Matrix3x3

    //ori_.setRPY(orientationRPY[0], orientationRPY[1], orientationRPY[2]);

    // NOTE USE EIGEN
    Eigen::Matrix<double, 3, 3> rotMat;
    rotMat = Eigen::AngleAxisd(orientationYPR[0], Eigen::Vector3d::UnitZ())
              * Eigen::AngleAxisd(orientationYPR[1], Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd(orientationYPR[2], Eigen::Vector3d::UnitX());

    tf::Matrix3x3 ori_( rotMat(0,0), rotMat(0,1), rotMat(0,2),
                        rotMat(1,0), rotMat(1,1), rotMat(1,2),
                        rotMat(2,0), rotMat(2,1), rotMat(2,2) );

    // Output tf::Transform
    tf::Transform tr(ori_, pos_);

    return tr;
}


void robotBehaviourManager::createThread( const unsigned int priority_,
                                          const int schedulingPolicy_,
                                          void *(*threadRoutine) (void *) )
{
 //   struct sched_param threadSchedulingParams;

    pthread_attr_t	threadAttributes;

    termintateTaskThread_ = false;
    threadCreated_		  = false;
    taskThreadReady_	  = false;

    pthread_mutex_init(&(this->mutexVar_), NULL);
    pthread_cond_init (&(this->condVar_), NULL);

    //threadSchedulingParams.sched_priority =	priority_;

    pthread_attr_init			(&threadAttributes);
    //pthread_attr_setschedpolicy	(&threadAttributes,	schedulingPolicy_);
    //pthread_attr_setinheritsched(&threadAttributes,	PTHREAD_EXPLICIT_SCHED);
    //pthread_attr_setschedparam	(&threadAttributes,	&threadSchedulingParams);

    pthread_create( &taskThread_,
                    &threadAttributes,
                    threadRoutine,
                    this);

    pthread_mutex_lock(&(this->mutexVar_));

    while (!threadCreated_){
        pthread_cond_wait(&(this->condVar_), &(this->mutexVar_));
    }

    pthread_mutex_unlock(&(this->mutexVar_));

    pthread_mutex_lock(&(this->mutexVar_));
    this->taskThreadReady_ = true;
    pthread_mutex_unlock(&(this->mutexVar_));

    pthread_cond_signal(&(this->condVar_));
}


void robotBehaviourManager::saveObjectList(std::string fileName){
    commInterp->objects->saveObjectListToXml(fileName);
}
