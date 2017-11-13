#include "kuka_seed_commands/commandInterpreter.h"

commandInterpreter::commandInterpreter( ros::NodeHandle node,
                                        std::string objectListFile,
                                        std::string seedToKukaTopic,
                                        std::string kukaToSeedTopic,
                                        std::string objectDistTopic,
                                        std::string gripperTopic,
                                        std::string objectsTopic ):cmdNode(node),
                                                                     lastSeedCommand(""),
                                                                     newObjectName("")

{
    pthread_mutex_init(&(this->callbackMtx), NULL);
    // Advertise gripper command topic
    grippCommPub = cmdNode.advertise<std_msgs::Int32>(gripperTopic, 0);
    // The first time topics are not readed
    std_msgs::Int32 cmd_;
    cmd_.data = 1;
    grippCommPub.publish(cmd_);

    // Advertise Robot to Seed topic
    kukaToSeedPub = cmdNode.advertise<std_msgs::String>(kukaToSeedTopic, 2);
    std_msgs::String tmpMsg;
    tmpMsg.data = "";
    kukaToSeedPub.publish(tmpMsg);

    // Advertise object distance topic
    objectDistPub = cmdNode.advertise<std_msgs::String>(objectDistTopic, 0);

    // Subscribe to the command topic from multimodal architecture
    seedToKukaSub = cmdNode.subscribe( seedToKukaTopic,
                                       0,
                                       &commandInterpreter::seedToKukaCallback,
                                       this );

    // tf listener
    tfListener = new tf::TransformListener(ros::Duration(10));

    // Create objects list
    std::string packPath = ros::package::getPath("kuka_seed_commands");
    objects = new objectsList(true, packPath + objectListFile);
    objectListFile.erase(objectListFile.end()-4, objectListFile.end());
    newObjectFile = packPath + objectListFile + "_new.xml";

    // Ar recognition/tracking subscriber
    arObjSub = cmdNode.subscribe( objectsTopic,
                                  0,
                                  &commandInterpreter::objTrackingCallback,
                                  this );

    robotCommand     = -1;
    gripperCommand   = -1;
    objectToUseId    = -1;
    objectToUseIndex = -1;

    actionToPerformIndex = -1;

    learnObjIndx   = 0;

    lastSegmentID  = 0;

    counterNewObject = 30;

    taskExecuted = false;
    isTeachMode  = false;

    closeCmdReceived = false;
    openCmdReceived  = false;
    preGraspPoseSetted = false;
    setPreGraspPose  = true;

    closestObjId      = -1;
    closestObjectDist = 1e8;

    publishMessage = false;

    cupCloseCounter = 0;
    closeToCup      = false;
    complexActionFound = false;

    // Init action struct
    std::vector<double> currPt(3, 0.0);
    newAction.label_ = "";
    newAction.frame_ = "";
    newAction.positions_.push_back(currPt);
    newAction.orientations_.push_back(currPt);

  //  arFrameName = "ar_marker_";

    //ros::spinOnce(); //not sure if needed
}


void commandInterpreter::objNameCallback(const std_msgs::StringConstPtr &namePtr){
    pthread_mutex_lock(&callbackMtx);
    newObjectName = namePtr->data;
    pthread_mutex_unlock(&callbackMtx);
}


void commandInterpreter::objTrackingCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &objPtr){
    counterNewObject--;
    if(counterNewObject<1){
        pthread_mutex_lock(&callbackMtx);
        int objIndex = -1;
        std_msgs::String objDists;
        std::stringstream  distsForSeed; distsForSeed << "(";
        std::string separator("");

        closestObjId = -1;
        closestObjectDist = 1e8;

        for(uint i=0; i<objPtr->markers.size(); i++){
            if((int)objPtr->markers[i].id>0){
                // Update objects pose
                if(objects->findObjectById((int)objPtr->markers[i].id, objIndex)){
                    objects->objectList[objIndex].poseWorldFrame = objPtr->markers[i].pose.pose;
                }
                else{ // Add unknown object to the list
                    objects->initObjectStructure(&object);
                    object.id             = (int)objPtr->markers[i].id;
                    object.poseWorldFrame = objPtr->markers[i].pose.pose;
                    // Update object list
                    objects->updateObjectList(object);

                    objIndex = (int)objects->objectList.size()-1;
                }

                // Compute Robot-Objects distances
                distsForSeed << separator << objects->objectList[objIndex].name;
                separator = ",";

                std::string arFrameName("ar_marker_");

                tf::StampedTransform tmpPose;
                int found = waitTfTransform(arFrameName + numToString(objPtr->markers[i].id), "wsg50_end_link", tmpPose);

                if(!found){
                    distsForSeed << separator << 1000.0;
                    ROS_ERROR( "Impossible to find a transformation from %s to %s \n",
                               (arFrameName + numToString((int)objPtr->markers[i].id)).c_str(), "wsg50_end_link");
                }
                else if(objects->objectList[objIndex].isKnown){
                    distsForSeed << separator << 1000.0;
                }
                else{
                    double dist_ = std::sqrt( tmpPose.getOrigin().x()*tmpPose.getOrigin().x() +
                                              tmpPose.getOrigin().y()*tmpPose.getOrigin().y() +
                                              tmpPose.getOrigin().z()*tmpPose.getOrigin().z() );

                    distsForSeed << separator << dist_;

                    if(dist_ < closestObjectDist){
                        closestObjectDist = dist_;
                        closestObjId      = objects->objectList[objIndex].id;
                    }
                }
            }
        }
        distsForSeed << ")";

        // Set pre-grasping pose
        bool teachOn = isTeachMode;
        if(teachOn){
            if(!closeCmdReceived && closestObjectDist<POS_THRESH && setPreGraspPose){
                // Set pre_take pose
                objectToUseId = closestObjId;
                if(setGrasingPose("ar_marker_")){
                    setPreGraspPose = false;
                    lastSegmentID++;
                    // Set pre-take position
                    int objIndx = 0;
                    objects->findObjectById(objectToUseId, objIndx);
                    for(int i=0; i<3; ++i){
                        objects->objectList[objIndx].preTakePosition[i]  = graspingPose.getOrigin()[i];
                    }

                    // Set take orientation
                    Eigen::Matrix<double, 3, 3> tmpRot;
                    tmpRot << graspingPose.getBasis()[0][0], graspingPose.getBasis()[0][1], graspingPose.getBasis()[0][2],
                            graspingPose.getBasis()[1][0], graspingPose.getBasis()[1][1], graspingPose.getBasis()[1][2],
                            graspingPose.getBasis()[2][0], graspingPose.getBasis()[2][1], graspingPose.getBasis()[2][2];

                    Eigen::Vector3d tmpAngles = rotationMatrixToRPY(tmpRot); //tmpRot.eulerAngles(2, 1, 0);

                    std::cout << "Pre-Take YPR: " << tmpAngles.transpose()*180.0/3.14 << std::endl;

                    for(int i=0; i<3; ++i)
                        objects->objectList[objIndx].preTakeOrientation[i] = tmpAngles(i);

                    // Set action
                    //newAction.label_ = "preTake" + numToString(lastSegmentID);
                    newAction.label_ = "foa" + numToString(lastSegmentID);
                    newAction.frame_ = objects->objectList[objIndx].name;
                    for(int i=0; i<3; ++i){
                        newAction.positions_[0][i]  = graspingPose.getOrigin()[i];
                        newAction.orientations_[0][i] = tmpAngles[i];
                    }
                    objects->objectList[objIndx].actions_.push_back(newAction);

                    std_msgs::String tmpMsg;
                    tmpMsg.data = "segment(" + newAction.label_ + ",true,true," + objects->objectList[objIndx].name + ")";
                    kukaToSeedPub.publish(tmpMsg);
                }
            }

            // Check object close to cup
            tf::StampedTransform transform, transformW;
            if(openCmdReceived && cupCloseCounter>1){ // Save if gripper open
                objects->objectList[objectToUseIndex].actions_.push_back(newAction);

                std_msgs::String tmpMsg;
                tmpMsg.data = "segment(" + newAction.label_ + ",true,true,worldComplex)";
                kukaToSeedPub.publish(tmpMsg);

                closeToCup = false;
                cupCloseCounter = 0;
               // complexActionFound = false;

                std::cout << "Learned Complex action. Size: " << newAction.positions_.size() << std::endl;

                // Clear for later use
                newAction.positions_.clear();
                newAction.orientations_.clear();

                std::vector<double> currPt(3, 0.0);
                newAction.positions_.push_back(currPt);
                newAction.orientations_.push_back(currPt);

            }
            else if(closeCmdReceived){ //Object in hand
                //if(waitTfTransform( "cup", "wsg50_end_link", transform )==1){
                if(waitTfTransform( "world", "cup", transform )==1){
                    // Compensate marker location
                    tf::Vector3 cupPos = transform.getOrigin();
                    cupPos.setY(cupPos.y() - 0.06);
                    cupPos.setZ(0.1);

                    transform.setOrigin(cupPos);
                    // Compute distance
                    double cupDist = 2.0*CUP_THRESH;
                    if(waitTfTransform( "world", "wsg50_end_link", transformW )==1){
                        double xd =  transform.getOrigin().x() -  transformW.getOrigin().x();
                        double yd =  transform.getOrigin().y() -  transformW.getOrigin().y();
                        double zd =  transform.getOrigin().z() -  transformW.getOrigin().z();

                        cupDist = std::sqrt(xd*xd + yd*yd + zd*zd);
                    }

                    /*double cupDist = std::sqrt( transform.getOrigin().x()*transform.getOrigin().x() +
                                                transform.getOrigin().y()*transform.getOrigin().y() +
                                                transform.getOrigin().z()*transform.getOrigin().z() );*/

                    if(cupDist<=CUP_THRESH){ // Store data
                        //ROS_WARN("Close to cup\n");
                        if(waitTfTransform( "world", "wsg50_end_link", transformW )==1){
                            cupCloseCounter++;
                            closeToCup = true;
                            if(cupCloseCounter == 1){
                                // Pre-complex pose
                                lastSegmentID++;
                                //newAction.label_ = "preComplex" + numToString(lastSegmentID);
                                newAction.label_ = "foa" + numToString(lastSegmentID);
                                newAction.frame_ = "world";

                                // Orientation to euler
                                Eigen::Vector3d tmpAngles = tfStampedTransformToRPY(transformW);

                                for(int i=0; i<3; ++i){
                                    newAction.positions_[0][i]  = transformW.getOrigin()[i];
                                    newAction.orientations_[0][i] = tmpAngles[i];
                                }

                                std::cout << "PreComplex angles: " << (180.0/3.14)*tmpAngles.transpose() << std::endl;

                                objects->objectList[objectToUseIndex].actions_.push_back(newAction);

                                std_msgs::String tmpMsg;
                                tmpMsg.data = "segment(" + newAction.label_ + ",true,true,world)";
                                kukaToSeedPub.publish(tmpMsg);

                                // Init Complex Action
                                lastSegmentID++;
                                //newAction.label_ = "complex" + numToString(lastSegmentID);
                                newAction.label_ = "noa" + numToString(lastSegmentID);
                                newAction.frame_ = "worldComplex";

                                complexActionFound = true;
                            }
                            else{ // Add training pose
                                newAction.positions_.push_back(std::vector<double>());
                                newAction.orientations_.push_back(std::vector<double>());

                                // Orientation to euler
                                Eigen::Vector3d tmpAngles = tfStampedTransformToRPY(transformW);

                              //  std::cout << "pre check angles: " << (180.0/3.1416)*tmpAngles.transpose() << std::endl;

                                // Check for singularities
                                checkSingularitiesRPY(newAction.orientations_[cupCloseCounter-2], tmpAngles);

                               // tmpAngles = (180.0/3.1416)*tmpAngles;

                              //  std::cout << "post check angles: " << (180.0/3.1416)*tmpAngles.transpose() << std::endl;

//                                if(cupCloseCounter==2){
//                                    for(int i=0; i<3; ++i){
//                                        newAction.orientations_[0][i] = (180.0/3.1416)*newAction.orientations_[0][i];
//                                    }
//                                }

                                for(int i=0; i<3; ++i){
                                    newAction.positions_[cupCloseCounter-1].push_back(transformW.getOrigin()[i]);
                                    newAction.orientations_[cupCloseCounter-1].push_back(tmpAngles[i]);
                                }
                            }
                        }
                        else{
                            ROS_ERROR( "Impossible to find a transformation from %s to %s \n","wsg50_end_link", "world");
                        }

                    }
                    else{
                        if(closeToCup){ // Publish learned action
                            objects->objectList[objectToUseIndex].actions_.push_back(newAction);

                            std_msgs::String tmpMsg;
                            tmpMsg.data = "segment(" + newAction.label_ + ",true,true,worldComplex)";
                            kukaToSeedPub.publish(tmpMsg);

                            std::cout << "Learned Complex action. Size: " << newAction.positions_.size() << std::endl;

                            // Clear for later use
                            newAction.positions_.clear();
                            newAction.orientations_.clear();

                            std::vector<double> currPt(3, 0.0);
                            newAction.positions_.push_back(currPt);
                            newAction.orientations_.push_back(currPt);
                        }
                        closeToCup = false;
                        complexActionFound = false;
                        cupCloseCounter = 0;
                    }
                }
                else{
                    ROS_ERROR( "Impossible to find a transformation from %s to %s \n","wsg50_end_link", "cup");
                }
            }
        }

        // Publish distance
        objDists.data = distsForSeed.str();
        objectDistPub.publish(objDists);

        counterNewObject = 10;
        pthread_mutex_unlock(&callbackMtx);
    }
}


void commandInterpreter::checkSingularitiesRPY(std::vector<double> oldAngles, Eigen::Vector3d &newAngles){
    // CHECK FOR SINGULARITIES +- 180deg
    /*double maxAngle = 170.0*3.1416/180.0;
    for(int i=0; i<3; ++i){
        if(newAngles(i)>maxAngle && oldAngles[i]<-0.0){
            newAngles(i) = -newAngles(i);
        }
        else if(newAngles(i)<-maxAngle && oldAngles[i]>0.0){
            newAngles(i) = -newAngles(i);
        }
    }*/

    double maxAngle = 90.0*3.1416/180.0;
    for(int i=0; i<3; ++i){
        if(newAngles(i)>maxAngle && oldAngles[i]<-0.0){
            newAngles(i) = newAngles(i) - 2.0*3.1416;
        }
        else if(newAngles(i)<-maxAngle && oldAngles[i]>0.0){
            newAngles(i) = newAngles(i) + 2.0*3.1416;
        }
    }
}


Eigen::Vector3d commandInterpreter::tfStampedTransformToRPY(tf::StampedTransform transform){
    Eigen::Matrix<double, 3, 3> tmpRot;
    tmpRot << transform.getBasis()[0][0], transform.getBasis()[0][1], transform.getBasis()[0][2],
              transform.getBasis()[1][0], transform.getBasis()[1][1], transform.getBasis()[1][2],
              transform.getBasis()[2][0], transform.getBasis()[2][1], transform.getBasis()[2][2];

    return rotationMatrixToRPY(tmpRot);
}


void commandInterpreter::gripperControl(std_msgs::Int32 gripperCommand_){
    grippCommPub.publish(gripperCommand_);
    // Update gripper command
    pthread_mutex_lock(&callbackMtx);
    if(gripperCommand_.data == 0)
        gripperCommand = commandInterpreter::CMD_CLOSE;
    else if(gripperCommand_.data == 2)
        gripperCommand = commandInterpreter::CMD_OPEN;

    pthread_mutex_unlock(&callbackMtx);
}


int commandInterpreter::setGrasingPose(std::string arFrameName){
    tf::StampedTransform tmpPose;
    int found = waitTfTransform(arFrameName + numToString(objectToUseId), "wsg50_end_link", tmpPose);

    if(!found){
        ROS_ERROR( "setGrasingPose: Impossible to find a transformation from %s to %s \n",
                   (arFrameName + numToString(objectToUseId)).c_str(), "wsg50_end_link");
    }
    else{
        graspingPose = tmpPose;

        ROS_INFO( "Grasping pose setted. Position: (%f %f %f)",
                  tmpPose.getOrigin().x(),
                  tmpPose.getOrigin().y(),
                  tmpPose.getOrigin().z() );
    }
    return found;
}


int commandInterpreter::getCurrentPose(tf::StampedTransform &tmpPose){
    int found = waitTfTransform("world", "wsg50_end_link", tmpPose);

    if(!found){
        ROS_ERROR( "CMD_TAKE: Impossible to find a transformation from %s to %s \n",
                   "wsg50_end_link", "world");
    }
    else{
        ROS_INFO( "Current Position: (%f %f %f)",
                  tmpPose.getOrigin().x(),
                  tmpPose.getOrigin().y(),
                  tmpPose.getOrigin().z() );
    }
    return found;
}


std::vector<std::string> commandInterpreter::seedCommandParser(std::string schemaInstance){
    bool isAtom=true, isString=false;
    char c;
    std::string app;
    std::vector<std::string> result;
    std::stringstream ss(schemaInstance);
    int count=0;
    //leggi il primo carattere della stringa
    ss>>c;
    //mentre non sei a fine stringa
    while(!ss.eof())
    {
        //se il carattere è un doppio apice e non sono in una stringa
        if(c=='"' && !isString){
            //allora sono in una stringa
            isString=true;
            //aggiungo l'apice
            app=app+c;
        }
        //se il carattere è un doppio apice e sono in una stringa
        else if(c=='"' && isString){
            //la stringa è finita
            isString=false;
            //aggiungo l'apice
            app=app+c;
            //aggiungila come elemento del funtore
            //result.push_back(app);
        }
        //mentre sono in una stringa
        else if(isString){
            //aggiungi il carattere senza controllarlo
            app=app+c;
        }
        //se sono un atomo ed il carattere letto è una parentesi aperta
        else if(c=='(' && isAtom){
            //non sono più un atomo
            isAtom=false;
            //inserisco il nome come primo elemento del vettore
            result.push_back(app);
            //pulisco la stringa d'appoggio
            app="";
            //salto la parentesi
            //            ss>>c;
        }
        else if(c=='('){
            count++;
            app=app+c;
        }
        else if(c==')'&& count!=0){
            count--;
            app=app+c;
        }
        //se il carattere letto non è una virgola
        else if(c!=',' || count!=0)
            //aggiungilo alla stringa d'appoggio
            app=app+c;
        //altrimenti (ie. il carattere è una virgola)
        else {
            //inserisci la stringa d'appoggio nel vettore risultato
            result.push_back(app);
            //pulisci la stringa d'appoggio
            app="";
            //ho saltato la virgola
        }
        //leggi il successivo carattere
        ss>>c;
    }
    //se lo schema non ha parametri aggiungi il solo nome (vec[0])
    if(isAtom) result.push_back(app);
    //altrimenti aggiungi l'ultima stringa rimuovendo l'ultima parentesi
    else{
        app.erase(app.size()-1);
        result.push_back(app);
    }
    //ritorna il vettore calcolato
    return result;
}


Eigen::Vector3d commandInterpreter::rotationMatrixToRPY(Eigen::Matrix<double, 3, 3> Rotation){
    Eigen::Vector3d rpy;

    rpy(0) = atan2(Rotation(1,0),Rotation(0,0));
    rpy(1) = atan2(-Rotation(2,0), sqrt(Rotation(2,1)*Rotation(2,1) + Rotation(2,2)*Rotation(2,2)));
    rpy(2) = atan2(Rotation(2,1),Rotation(2,2));

    return rpy;
}


void commandInterpreter::seedToKukaCallback(const std_msgs::String::ConstPtr &cmdPtr){
    // Store received command
    newSeedCommand = cmdPtr->data;
    std::string seedAction("");
    std::string seedObject("");

    if(lastSeedCommand != newSeedCommand ){
        pthread_mutex_lock(&callbackMtx);
        taskExecuted = false;
        pthread_mutex_unlock(&callbackMtx);

        publishMessage = true;

        lastSeedCommand = newSeedCommand;
#ifdef DEBUG_MODE
        std::cout << "Command received: " << newSeedCommand.c_str() << std::endl;
#endif

        // Parse received message
        std::string action("");
        std::vector<std::string> res_ = seedCommandParser(newSeedCommand);

        action = res_[0];
        if(res_.size()>1){
            seedAction = res_[1];
        }

        int curCom = -1;

        pthread_mutex_lock(&callbackMtx);
        // Decode current action
        if(action == "execute"){
            taskExecuted = false;
            setPreGraspPose = false;
            seedObject   = res_[2];
            std::cout << action << " " << seedAction << " " << seedObject << std::endl;
            if(seedObject != "world" && seedObject != "worldComplex"){
                if(objects->findObjectByName(seedObject, objectToUseIndex)){
                    objectToUseId = objects->objectList[objectToUseIndex].id;
                }
                else{
                    ROS_ERROR("Object: %s not found!\n", seedObject.c_str());
                    objectToUseId = -1;
                    objectToUseIndex = -1;
                    actionToPerformIndex = -1;
                }
            }

            if(objectToUseIndex > -1){
                int dim_ = (int)objects->objectList[objectToUseIndex].actions_.size();
                for(int j=0; j<dim_; ++j){
                    std::cout << objects->objectList[objectToUseIndex].actions_[j].label_ << std::endl;
                    if(objects->objectList[objectToUseIndex].actions_[j].label_ == seedAction){
                        actionToPerformIndex = j;
                        std::cout << "actionToPerformIndex setted" << std::endl;
                    }
                }

                robotCommand = CMD_EXECUTE;
            }
        }
        else if(action == "stop"){
            publishMessage = false;
            robotCommand = CMD_STOP;
        }
        else if(action == "repeat"){
            robotCommand = CMD_START;
        }
        /*else if(action == "place"){
            robotCommand = CMD_PLACE;
            curCom       = CMD_PLACE;
        }
        else if(action == "give"){
            robotCommand = CMD_SHOW;
        }*/
        else if(action == "teach"){
            robotCommand = CMD_TEACH;
        }
        else if(action == "done"){// Publish also object
            robotCommand = CMD_DONE;
            for(uint i=0; i<objects->objectList.size(); ++i)
                objects->objectList[i].isKnown = false;

            gripperCommand = -1;
        }
        else if(action == "shutdown"){// terminate fri and program
            robotCommand = CMD_TERMIN;
            std_msgs::String tmpMsg;
            tmpMsg.data = action;
            kukaToSeedPub.publish(tmpMsg);
        }
        else if(action == "gripper"){// terminate fri and program
            ;
        }
        else{
            ROS_WARN("Action: %s unknown!\n", action.c_str());
            robotCommand = -1;
            gripperCommand = -1;
        }
        pthread_mutex_unlock(&callbackMtx);

        // Find object to use
        /*if(curCom==CMD_EXECUTE){
            std::cout << action << " " << seedAction << " " << seedObject << std::endl;
            if(seedObject != "world"){
                pthread_mutex_lock(&callbackMtx);
                if(objects->findObjectByName(seedObject, objectToUseIndex)){
                    objectToUseId = objects->objectList[objectToUseIndex].id;
                }
                else{
                    ROS_ERROR("Object: %s not found!\n", seedObject.c_str());
                    objectToUseId = -1;
                    objectToUseIndex = -1;
                    actionToPerformIndex = -1;
                }

                int dim_ = (int)objects->objectList[objectToUseIndex].actions_.size();
                for(int j=0; j<dim_; ++j){
                    if(objects->objectList[objectToUseIndex].actions_[j].label_ == seedAction){
                        actionToPerformIndex = j;
                        std::cout << "actionToPerformIndex setted" << std::endl;
                    }
                }

                pthread_mutex_unlock(&callbackMtx);
            }
        }
        else */if(seedAction == "open"){
            // Set release pose
            // Assume objectToUseIndex already setted (always after grippr close)
            pthread_mutex_lock(&callbackMtx);
            bool teachOn = isTeachMode;
            closeCmdReceived = false;
            openCmdReceived = true;
            if(teachOn){
                objectToUseId = objects->objectList[objectToUseIndex].id;

                if(getCurrentPose(releasePose)){
                    // Set place position
                    for(int i=0; i<3; ++i){
                        objects->objectList[objectToUseIndex].placePosition[i]  = releasePose.getOrigin()[i];
                    }
                }

                // Set place orientation
                bool complexAction = complexActionFound;
                Eigen::Matrix<double, 3, 3> tmpRot;
                tmpRot << releasePose.getBasis()[0][0], releasePose.getBasis()[0][1], releasePose.getBasis()[0][2],
                        releasePose.getBasis()[1][0], releasePose.getBasis()[1][1], releasePose.getBasis()[1][2],
                        releasePose.getBasis()[2][0], releasePose.getBasis()[2][1], releasePose.getBasis()[2][2];

                Eigen::Vector3d tmpAngles = rotationMatrixToRPY(tmpRot);
                for(int i=0; i<3; ++i){
                    objects->objectList[objectToUseIndex].placeOrientation[i] = tmpAngles(i);
                }

                // Set pre-place position
                if(!complexAction){
                    for(int i=0; i<3; ++i){
                        objects->objectList[objectToUseIndex].prePlacePosition[i] = releasePose.getOrigin()[i];
                    }
                    objects->objectList[objectToUseIndex].prePlacePosition[2] += ACTION_OFFSET;

                    // Set pre-place orientation
                    for(int i=0; i<3; ++i){
                        objects->objectList[objectToUseIndex].prePlaceOrientation[i] = tmpAngles(i);
                    }

                    // Set pre-place action
                    std_msgs::String tmpMsg;
                    lastSegmentID++;
                    //tmpMsg.data = "segment(prePlace" + numToString(lastSegmentID) + ",true,true,world)";
                    tmpMsg.data = "segment(foa" + numToString(lastSegmentID) + ",true,true,world)";
                    kukaToSeedPub.publish(tmpMsg);
                    sleep(1);

                    //newAction.label_ = "prePlace" + numToString(lastSegmentID);
                    newAction.label_ = "foa" + numToString(lastSegmentID);
                    newAction.frame_ = "world";
                    for(int i=0; i<3; ++i){
                        newAction.positions_[0][i]  = releasePose.getOrigin()[i];
                        newAction.orientations_[0][i] = tmpAngles[i];
                    }
                    newAction.positions_[0][2] += ACTION_OFFSET;
                    objects->objectList[objectToUseIndex].actions_.push_back(newAction);

                    // Publish place action
                    lastSegmentID++;
                    //tmpMsg.data = "segment(a" + numToString(lastSegmentID) + ",true,true,world)";
                    tmpMsg.data = "segment(foa" + numToString(lastSegmentID) + ",true,true,world)";
                    kukaToSeedPub.publish(tmpMsg);

                    // Set place action
                    //newAction.label_ = "a" + numToString(lastSegmentID);
                    newAction.label_ = "foa" + numToString(lastSegmentID);
                    newAction.frame_ = "world";
                    for(int i=0; i<3; ++i){
                        newAction.positions_[0][i]  = releasePose.getOrigin()[i];
                        newAction.orientations_[0][i] = tmpAngles[i];
                    }
                    objects->objectList[objectToUseIndex].actions_.push_back(newAction);
                }
                else{

                    if(closeToCup){
                        pthread_mutex_unlock(&callbackMtx);
                        lastSeedCommand = "";
                        return;
                    }
                    else{
                        complexActionFound = false;
                    }
                    /*bool waitComplexAction = true;
                    while(waitComplexAction){
                        waitComplexAction = closeToCup;
                        pthread_mutex_unlock(&callbackMtx);
                        usleep(30000);
                        std::cout << "here" << std::endl;
                        pthread_mutex_lock(&callbackMtx);
                    }
                    complexActionFound = false;
                    setPreGraspPose = false;*/
                }


                // Set post-place pose
                //double offSign = 0.0;
                //int axis_ = checkGripperAlignment(offSign); //WORKS BUT STUPID
                int axis_ = 1;
                if(axis_<0){
                    ROS_ERROR("Post-place pose not setted!!\n");
                }
                else{
                    // Set post-place position and orientation
                    for(int i=0; i<3; ++i){
                        objects->objectList[objectToUseIndex].postPlacePosition[i] = 0.0;
                        objects->objectList[objectToUseIndex].postPlaceOrientation[i] = 0.0;
                    }
                    objects->objectList[objectToUseIndex].postPlacePosition[2] += ACTION_OFFSET;


                    // Set post-place action
                    newAction.label_ = "postPlace";
                    newAction.frame_ = "worldDelta";
                    for(int i=0; i<3; ++i){
                        newAction.positions_[0][i]  = 0.0;
                        newAction.orientations_[0][i] = 0.0;
                    }
                    newAction.positions_[0][2] += ACTION_OFFSET;
                    objects->objectList[objectToUseIndex].actions_.push_back(newAction);
                }
            }
            pthread_mutex_unlock(&callbackMtx);

            // Open Gripper
            std_msgs::Int32 gripperCommand_;
            gripperCommand_.data = 2;
            gripperControl(gripperCommand_);
            sleep(3);
            //taskExecuted = true;
            setPreGraspPose = true;

            // Post-Place execution
            if(!teachOn){
                int dim_ = (int)objects->objectList[objectToUseIndex].actions_.size();
                for(int j=0; j<dim_; ++j){
                    std::cout << objects->objectList[objectToUseIndex].actions_[j].label_ << std::endl;
                    if(objects->objectList[objectToUseIndex].actions_[j].label_ == "postPlace"){
                        actionToPerformIndex = j;
                        std::cout << "actionToPerformIndex setted" << std::endl;
                    }
                }
                robotCommand = CMD_EXECUTE;
            }
        }
        else if(seedAction == "close"){
            // Set take pose
            pthread_mutex_lock(&callbackMtx);
            bool teachOn = isTeachMode;
            pthread_mutex_unlock(&callbackMtx);
            if(teachOn){
                if(findClosestObject(objectToUseIndex)==0){
                    objectToUseId = objects->objectList[objectToUseIndex].id;
                    objects->objectList[objectToUseIndex].isKnown = true;
                    if(setGrasingPose("ar_marker_")){
                        // Set take position
                        for(int i=0; i<3; ++i){
                            objects->objectList[objectToUseIndex].takePosition[i]  = graspingPose.getOrigin()[i];
                        }

                        // Set take orientation
                        Eigen::Matrix<double, 3, 3> tmpRot;
                        tmpRot << graspingPose.getBasis()[0][0], graspingPose.getBasis()[0][1], graspingPose.getBasis()[0][2],
                                graspingPose.getBasis()[1][0], graspingPose.getBasis()[1][1], graspingPose.getBasis()[1][2],
                                graspingPose.getBasis()[2][0], graspingPose.getBasis()[2][1], graspingPose.getBasis()[2][2];

                        Eigen::Vector3d tmpAngles = rotationMatrixToRPY(tmpRot); //tmpRot.eulerAngles(2, 1, 0);

                        std::cout << "desired YPR: " << tmpAngles.transpose()*180.0/3.14 << std::endl;

                        for(int i=0; i<3; ++i)
                            objects->objectList[objectToUseIndex].takeOrientation[i] = tmpAngles(i);

                        // Set post-take position
                        for(int i=0; i<3; ++i){
                            objects->objectList[objectToUseIndex].postTakePosition[i] = graspingPose.getOrigin()[i];
                        }
                        objects->objectList[objectToUseIndex].postTakePosition[2] += ACTION_OFFSET;

                        // Set post-take orientation
                        for(int i=0; i<3; ++i){
                            objects->objectList[objectToUseIndex].postTakeOrientation[i] = tmpAngles(i);
                        }

                        // Set take action
                        lastSegmentID++;
                        //newAction.label_ = "a" + numToString(lastSegmentID);
                        newAction.label_ = "noa" + numToString(lastSegmentID);
                        newAction.frame_ = objects->objectList[objectToUseIndex].name;
                        for(int i=0; i<3; ++i){
                            newAction.positions_[0][i]  = graspingPose.getOrigin()[i];
                            newAction.orientations_[0][i] = tmpAngles[i];
                        }
                        objects->objectList[objectToUseIndex].actions_.push_back(newAction);

                        std_msgs::String tmpMsg;
                        //tmpMsg.data = "segment(a" + numToString(lastSegmentID) + ",true,true," + objects->objectList[objectToUseIndex].name + ")";
                        tmpMsg.data = "segment(noa" + numToString(lastSegmentID) + ",true,true," + objects->objectList[objectToUseIndex].name + ")";
                        kukaToSeedPub.publish(tmpMsg);

                        // Set post-take action (local - no seed)
                        //newAction.label_ = "postTake" + numToString(lastSegmentID+1);
                        newAction.label_ = "foa" + numToString(lastSegmentID+1);
                        newAction.frame_ = "worldDelta";
                        for(int i=0; i<3; ++i){
                            newAction.positions_[0][i]  = 0.0;
                            newAction.orientations_[0][i] = 0.0;
                        }
                        newAction.positions_[0][2] = ACTION_OFFSET; // Delta Action
                        objects->objectList[objectToUseIndex].actions_.push_back(newAction);
                    }
                }
            }

            // Close gripper
            std_msgs::Int32 gripperCommand_;
            gripperCommand_.data = 0;
            gripperControl(gripperCommand_);
            sleep(3);
            taskExecuted = true;
            closeCmdReceived = true;
            openCmdReceived = false;
            setPreGraspPose  = false;
        }
    }

    pthread_mutex_lock(&callbackMtx);
    if(taskExecuted && publishMessage){
        taskExecuted = false;
        publishMessage = false;
        std_msgs::String tmpMsg;

        //if(lastSeedCommand == "done"){
        //    tmpMsg.data = lastSeedCommand + "(" + objects->objectList[objectToUseIndex].name + ")";
        //    kukaToSeedPub.publish(tmpMsg);
        //}
        //else
        if(lastSeedCommand == "gripper(close)"){
            tmpMsg.data = "gripper(close," + objects->objectList[objectToUseIndex].name + ")";
            kukaToSeedPub.publish(tmpMsg);

            if(isTeachMode){
                sleep(1);
                std_msgs::String tmpMsg;
                lastSegmentID++;
                //tmpMsg.data = "segment(postTake" + numToString(lastSegmentID) + ",true,true,world)";
                tmpMsg.data = "segment(foa" + numToString(lastSegmentID) + ",true,true,world)";
                kukaToSeedPub.publish(tmpMsg);
                sleep(1);
            }
        }
        else{
            tmpMsg.data = lastSeedCommand;
            kukaToSeedPub.publish(tmpMsg);
        }
    }
    pthread_mutex_unlock(&callbackMtx);
}

// OLD version
/*void commandInterpreter::seedToKukaCallback(const std_msgs::String::ConstPtr &cmdPtr){
    // Store received command
    newSeedCommand = cmdPtr->data;
    std::string seedObject("");

    if(lastSeedCommand != newSeedCommand ){
        pthread_mutex_lock(&callbackMtx);
        taskExecuted = false;
        pthread_mutex_unlock(&callbackMtx);

        publishMessage = true;

        lastSeedCommand = newSeedCommand;
#ifdef DEBUG_MODE
        std::cout << "Command received: " << newSeedCommand.c_str() << std::endl;
#endif

        // Parse received message
        std::string action("");
        std::vector<std::string> res_ = seedCommandParser(newSeedCommand);

        action = res_[0];
        if(res_.size()>1){
            seedObject = res_[1];
        }

        int curCom = -1;

        pthread_mutex_lock(&callbackMtx);
        // Decode current action
        if(action == "take"){
            robotCommand = CMD_TAKE;
            curCom       = CMD_TAKE;
        }
        else if(action == "place"){
            robotCommand = CMD_PLACE;
            curCom       = CMD_PLACE;
        }
        else if(action == "stop"){
            robotCommand = CMD_STOP;
        }
        else if(action == "give"){
            robotCommand = CMD_SHOW;
        }
        else if(action == "teach"){
            robotCommand = CMD_TEACH;
        }
        else if(action == "done"){// Publish also object
            robotCommand = CMD_DONE;
            gripperCommand = -1;
        }
        else if(action == "shutdown"){// terminate fri and program
            robotCommand = CMD_TERMIN;
            std_msgs::String tmpMsg;
            tmpMsg.data = action;
            kukaToSeedPub.publish(tmpMsg);
        }
        else if(action == "gripper"){// terminate fri and program
            ;
        }
        else{
            ROS_WARN("Action: %s unknown!\n", action.c_str());
            robotCommand = -1;
            gripperCommand = -1;
        }
        pthread_mutex_unlock(&callbackMtx);

        // Find object to use
        if((curCom==CMD_TAKE) || (curCom==CMD_PLACE)){
            int objInd;
            if(objects->findObjectByName(seedObject, objInd)){
                objectToUseId = objects->objectList[objInd].id;
            }
            else{
                ROS_ERROR("Object: %s not found!\n", seedObject.c_str());
                objectToUseId = -1;
            }
        }
        else if(seedObject == "open"){
            // Set release pose
            // Assume objectToUseIndex already setted (always after grippr close)
            objectToUseId = objects->objectList[objectToUseIndex].id;

            if(getCurrentPose(releasePose)){
                // Set place position
                for(int i=0; i<3; ++i){
                    objects->objectList[objectToUseIndex].placePosition[i]  = releasePose.getOrigin()[i];
                }
            }

            // Set place orientation
            Eigen::Matrix<double, 3, 3> tmpRot;
            tmpRot << releasePose.getBasis()[0][0], releasePose.getBasis()[0][1], releasePose.getBasis()[0][2],
                      releasePose.getBasis()[1][0], releasePose.getBasis()[1][1], releasePose.getBasis()[1][2],
                      releasePose.getBasis()[2][0], releasePose.getBasis()[2][1], releasePose.getBasis()[2][2];

            Eigen::Vector3d tmpAngles = rotationMatrixToRPY(tmpRot);
            for(int i=0; i<3; ++i){
                objects->objectList[objectToUseIndex].placeOrientation[i] = tmpAngles(i);
            }

            // Set pre-place position
            for(int i=0; i<3; ++i){
                objects->objectList[objectToUseIndex].prePlacePosition[i] = releasePose.getOrigin()[i];
            }
            objects->objectList[objectToUseIndex].prePlacePosition[2] += ACTION_OFFSET;

            // Set pre-place orientation
            for(int i=0; i<3; ++i){
                objects->objectList[objectToUseIndex].prePlaceOrientation[i] = tmpAngles(i);
            }


            // Set post-place pose
            double offSign = 0.0;
            int axis_ = checkGripperAlignment(offSign);
            if(axis_<0){
                ROS_ERROR("Post-place pose not setted!!\n");
            }
            else{
                // Set post-place position
                for(int i=0; i<3; ++i){
                    objects->objectList[objectToUseIndex].postPlacePosition[i] = releasePose.getOrigin()[i];
                }
                objects->objectList[objectToUseIndex].postPlacePosition[axis_] += offSign*ACTION_OFFSET;

                // Set post-place orientation
                for(int i=0; i<3; ++i){
                    objects->objectList[objectToUseIndex].postPlaceOrientation[i] = tmpAngles(i);
                }

                std_msgs::String tmpMsg;
                lastSegmentID++;
                tmpMsg.data = "segment(a" + numToString(lastSegmentID) + ",false,true,world)";
                kukaToSeedPub.publish(tmpMsg);
            }

            // Open Gripper
            std_msgs::Int32 gripperCommand_;
            gripperCommand_.data = 2;
            gripperControl(gripperCommand_);
            sleep(3);
            taskExecuted = true;
            closeCmdReceived = false;
            setPreGraspPose = true;
        }
        else if(seedObject == "close"){
            // Set take pose
            if(findClosestObject(objectToUseIndex)==0){
                objectToUseId = objects->objectList[objectToUseIndex].id;
                if(setGrasingPose("ar_marker_")){
                    // Set take position
                    for(int i=0; i<3; ++i){
                        objects->objectList[objectToUseIndex].takePosition[i]  = graspingPose.getOrigin()[i];
                    }

                    // Set take orientation
                    Eigen::Matrix<double, 3, 3> tmpRot;
                    tmpRot << graspingPose.getBasis()[0][0], graspingPose.getBasis()[0][1], graspingPose.getBasis()[0][2],
                              graspingPose.getBasis()[1][0], graspingPose.getBasis()[1][1], graspingPose.getBasis()[1][2],
                              graspingPose.getBasis()[2][0], graspingPose.getBasis()[2][1], graspingPose.getBasis()[2][2];

                    Eigen::Vector3d tmpAngles = rotationMatrixToRPY(tmpRot); //tmpRot.eulerAngles(2, 1, 0);

                    std::cout << "desired YPR: " << tmpAngles.transpose()*180.0/3.14 << std::endl;

                    for(int i=0; i<3; ++i)
                        objects->objectList[objectToUseIndex].takeOrientation[i] = tmpAngles(i);

                    // Set post-take position
                    for(int i=0; i<3; ++i){
                        objects->objectList[objectToUseIndex].postTakePosition[i] = graspingPose.getOrigin()[i];
                    }
                    objects->objectList[objectToUseIndex].postTakePosition[2] += ACTION_OFFSET;

                    // Set post-take orientation
                    for(int i=0; i<3; ++i){
                        objects->objectList[objectToUseIndex].postTakeOrientation[i] = tmpAngles(i);
                    }

                    std_msgs::String tmpMsg;
                    lastSegmentID++;
                    tmpMsg.data = "segment(a" + numToString(lastSegmentID) + ",true,true," + objects->objectList[objectToUseIndex].name + ")";
                    kukaToSeedPub.publish(tmpMsg);
                }
            }

            // Close gripper
            std_msgs::Int32 gripperCommand_;
            gripperCommand_.data = 0;
            gripperControl(gripperCommand_);
            sleep(3);
            taskExecuted = true;
            closeCmdReceived = true;
            setPreGraspPose  = false;
        }
    }

    pthread_mutex_lock(&callbackMtx);
    if(taskExecuted && publishMessage){
        taskExecuted = false;
        publishMessage = false;
        std_msgs::String tmpMsg;

        //if(lastSeedCommand == "done"){
        //    tmpMsg.data = lastSeedCommand + "(" + objects->objectList[objectToUseIndex].name + ")";
        //    kukaToSeedPub.publish(tmpMsg);
        //}
        //else
        if(lastSeedCommand == "gripper(close)"){
            tmpMsg.data = "gripper(close," + objects->objectList[objectToUseIndex].name + ")";
            kukaToSeedPub.publish(tmpMsg);
        }
        else{
            tmpMsg.data = lastSeedCommand;
            kukaToSeedPub.publish(tmpMsg);
        }
    }
    pthread_mutex_unlock(&callbackMtx);
}*/


int commandInterpreter::checkGripperAlignment(double &offsetDign){
    // Current gripper pose
    tf::StampedTransform tmpPose;
    int found = waitTfTransform("world", "wsg50_end_link", tmpPose);

    if(!found){
        ROS_ERROR( "setGrasingPose: Impossible to find a transformation from %s to %s \n",
                   "wsg50_end_link", "world");

        return -1;
    }

    // Gripper Z-axis
    Eigen::Vector3d gripperZ;
    gripperZ << tmpPose.getBasis()[0][2], tmpPose.getBasis()[1][2], tmpPose.getBasis()[2][2];
    // World frame
    Eigen::Vector3d wX, wY, wZ;
    wX << 1.0, 0.0, 0.0;
    wY << 0.0, 1.0, 0.0;
    wZ << 0.0, 0.0, 1.0;

    // Compute angle cosine
    double cosX = gripperZ.dot(wX), cosY = gripperZ.dot(wY), cosZ = gripperZ.dot(wZ);
    double aCosX = std::abs(cosX), aCosY = std::abs(cosY), aCosZ = std::abs(cosZ);

    // Find most aligned world frame axis
    int axis_ = 0;
    offsetDign = 1.0;
    if((aCosY > aCosX) && (aCosY > aCosZ)){
        axis_ = 1;
        offsetDign = -1.0;
    }
    else if((aCosZ > aCosX) && (aCosZ > aCosY)){
        axis_ = 2;
        if(aCosY>=0.0)
            offsetDign = 1.0;
        else
            offsetDign = -1.0;
    }

    std::cout << "DEBUG: axis number " << axis_ << std::endl;

    return axis_;
}


void commandInterpreter::updateLearnedObject(bool verbouse_){
    // Update object in the list
    if(!newObjectName.empty()){
        objects->objectList[learnObjIndx].name = newObjectName;
        newObjectName.clear();
    }
    else{
        ROS_WARN("Object name not setted! \n");
    }

    // Set grasp/release positions
    for(int i=0; i<3; ++i){
        objects->objectList[learnObjIndx].takePosition[i]  = graspingPose.getOrigin()[i];
        objects->objectList[learnObjIndx].placePosition[i] = releasePose.getOrigin()[i];
    }
    // Adding z_axis offset to avoid to collide with objects
    //objects->objectList[learnObjIndx].takePosition[2]  += Z_AXIZ_OFFSET;
    //objects->objectList[learnObjIndx].placePosition[2] += Z_AXIZ_OFFSET;

    // Set grasp/release orientations
   /* tf::Matrix3x3(graspingPose.getRotation()).getRPY( objects->objectList[learnObjIndx].takeOrientation[0],
                                                      objects->objectList[learnObjIndx].takeOrientation[1],
                                                      objects->objectList[learnObjIndx].takeOrientation[2] );

    tf::Matrix3x3(releasePose.getRotation()).getRPY( objects->objectList[learnObjIndx].placeOrientation[0],
                                                     objects->objectList[learnObjIndx].placeOrientation[1],
                                                     objects->objectList[learnObjIndx].placeOrientation[2] );*/

    // NOTE USE EIGEN
    // Take
    Eigen::Matrix<double, 3, 3> tmpRot;
    tmpRot << graspingPose.getBasis()[0][0], graspingPose.getBasis()[0][1], graspingPose.getBasis()[0][2],
              graspingPose.getBasis()[1][0], graspingPose.getBasis()[1][1], graspingPose.getBasis()[1][2],
              graspingPose.getBasis()[2][0], graspingPose.getBasis()[2][1], graspingPose.getBasis()[2][2];

    Eigen::Vector3d tmpAngles = tmpRot.eulerAngles(2, 1, 0);

    for(int i=0; i<3; ++i)
        objects->objectList[learnObjIndx].takeOrientation[i] = tmpAngles(i);

    // Place
    tmpRot << releasePose.getBasis()[0][0], releasePose.getBasis()[0][1], releasePose.getBasis()[0][2],
              releasePose.getBasis()[1][0], releasePose.getBasis()[1][1], releasePose.getBasis()[1][2],
              releasePose.getBasis()[2][0], releasePose.getBasis()[2][1], releasePose.getBasis()[2][2];

    tmpAngles = tmpRot.eulerAngles(2, 1, 0);

    for(int i=0; i<3; ++i)
        objects->objectList[learnObjIndx].placeOrientation[i] = tmpAngles(i);

    objects->objectList[learnObjIndx].isKnown = true;

    if(verbouse_){
        objects->printObjectInList(learnObjIndx);
    }
}


int commandInterpreter::findClosestObject(int &objectIndex){
    double dist_ = 10000.0, tmpDist = 0.0;
    objectIndex  = -1;
    // Read robot end-effector pose
    tf::StampedTransform robotToWorld;
    if(!waitTfTransform("camera_rgb_optical_frame", "wsg50_end_link", robotToWorld)){
        return -1;
    }

    double xo, yo, zo, xr, yr, zr;
    // Robot position
    xr = robotToWorld.getOrigin().x();
    yr = robotToWorld.getOrigin().y();
    zr = robotToWorld.getOrigin().z();
    for(uint i=0; i<objects->objectList.size(); ++i){
        // Object i-th position
        if(objects->objectList[i].name != "tray"){
            xo = objects->objectList[i].poseWorldFrame.position.x;
            yo = objects->objectList[i].poseWorldFrame.position.y;
            zo = objects->objectList[i].poseWorldFrame.position.z;

            // Squared distance
            tmpDist = (xr-xo)*(xr-xo) + (yr-yo)*(yr-yo) + (zr-zo)*(zr-zo);

            if(tmpDist < dist_){
                dist_       = tmpDist;
                objectIndex = i;
            }
        }
    }
    return 0;
}


int commandInterpreter::waitTfTransform( std::string targetFrame,
                                         std::string sourceFrame,
                                         tf::StampedTransform &transform )
{
    ros::Time curr_time;
    int founded = 0;
    if(tfListener->waitForTransform(targetFrame, sourceFrame, curr_time, ros::Duration(0.03))){
        tfListener->lookupTransform(targetFrame, sourceFrame, curr_time, transform);
        founded = 1;
    }
    return founded;
}


std::string commandInterpreter::numToString(int number){
    std::stringstream s;
    s << number;
    return s.str();
}


/*int commandInterpreter::findClosestTray(int &objectIndex){
    double dist_ = 10000.0, tmpDist = 0.0;
    objectIndex  = -1;
    // Read robot end-effector pose
    tf::StampedTransform robotToWorld;
    if(!waitTfTransform("world", "wsg50_end_link", robotToWorld)){
        return -1;
    }

    double xo, yo, zo, xr, yr, zr;
    // Robot position
    xr = robotToWorld.getOrigin().x();
    yr = robotToWorld.getOrigin().y();
    zr = robotToWorld.getOrigin().z();
    for(uint i=0; i<objects->objectList.size(); ++i){
        if(objects->objectList[i].name == "tray"){
            // i-th tray position
            xo = objects->objectList[i].poseWorldFrame.position.x;
            yo = objects->objectList[i].poseWorldFrame.position.y;
            zo = objects->objectList[i].poseWorldFrame.position.z;

            // Squared distance
            tmpDist = (xr-xo)*(xr-xo) + (yr-yo)*(yr-yo) + (zr-zo)*(zr-zo);

            if(tmpDist < dist_){
                dist_       = tmpDist;
                objectIndex = i;
            }
        }
    }
    return 0;
}*/
