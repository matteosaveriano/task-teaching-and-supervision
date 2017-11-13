#include "kuka_seed_commands/objectsList.h"


objectsList::objectsList( bool loadObjectXmlList,
                          std::string xmlFileName )
{
    initObjectStructure(&newObject);

    objectXmlFile = new TiXmlDocument();

    loadXmlList = loadObjectXmlList;
    if(loadXmlList){
        if(!createObjectListFromXml(xmlFileName)){
            std::cout << "Error while loading xml object database" << std::endl;
            std::cout << xmlFileName << " not found!" << std::endl;
        }
    }

    objRootElement = NULL;
}


objectsList::~objectsList(){
    if(loadXmlList){
        delete objectXmlFile;
        delete objRootElement;
    }
}


bool objectsList::findObjectById(int objectID, int &objectIndex){
    int listDim = (int)objectList.size(), i = 0;
    bool objectFound = false;
    objectIndex = -1;
    objectsList::objectsDescription tmpObject;

    while(!objectFound && i<listDim){
        tmpObject = getObjectByIndex(i);
        if(tmpObject.id == objectID){
            objectFound = true;
            objectIndex = i;
        }
        ++i;
    }
    return objectFound;
}


bool objectsList::findObjectByName(std::string objectName, int &objectIndex){
    int listDim = (int)objectList.size(), i = 0;
    bool objectFound = false;
    objectIndex = -1;
    objectsList::objectsDescription tmpObject;

    while(!objectFound && i<listDim){
        tmpObject = getObjectByIndex(i);
        if(tmpObject.name == objectName){
            objectFound = true;
            objectIndex = i;
        }
        ++i;
    }
    return objectFound;
}


bool objectsList::getObjectByID(int objectID, objectsList::objectsDescription *object){
    int objectIndex = -1;
    bool objectFound = findObjectById(objectID, objectIndex);
    // Create an empty object structure
    initObjectStructure(object);

    if(objectFound){
        *object = getObjectByIndex(objectIndex);
    }
    return objectFound;
}


void objectsList::initObjectStructure(objectsList::objectsDescription *object){
    object->id   = -1;
    object->name = "";
    object->graspForce   = 0.0;
    // Allocate space for the new object
    for(int i=0; i<3; ++i){
        object->preTakePosition.push_back(0.0);
        object->preTakeOrientation.push_back(0.0);
        object->postTakePosition.push_back(0.0);
        object->postTakeOrientation.push_back(0.0);
        object->takePosition.push_back(0.0);
        object->takeOrientation.push_back(0.0);
        object->placePosition.push_back(0.0);
        object->placeOrientation.push_back(0.0);
        object->prePlacePosition.push_back(0.0);
        object->prePlaceOrientation.push_back(0.0);
        object->postPlacePosition.push_back(0.0);
        object->postPlaceOrientation.push_back(0.0);
    }
    object->isKnown = false;

    // Init action struct
    /*std::vector<double> currPt(3, 0.0);
    objectsList::actionDescriptor emptyAction;
    emptyAction.label_ = "";
    emptyAction.frame_ = "";
    emptyAction.positions_.push_back(currPt);
    emptyAction.orientations_.push_back(currPt);

    object->actions_.push_back(emptyAction);*/
}


bool objectsList::loadObjectListFromXml(std::string xmlFileName){
    // Load Xml document
    bool fileLoaded = objectXmlFile->LoadFile(xmlFileName);
    if(!fileLoaded){
        return fileLoaded;
    }

    // Root node in the document
    objRootElement = objectXmlFile->FirstChildElement();
    if(!objRootElement){
        return false;
    }

    return fileLoaded;
}


void objectsList::printObjectList(){
    int listDim = (int)objectList.size(), i = 0;
    for(i=0; i<listDim; ++i){
        printObjectInList(i);
    }
}


void objectsList::printObjectInList(int objectIndex){
    std::cout << "Object " << objectIndex << ":" << std::endl;
    std::cout << "\t Name: " << objectList[objectIndex].name << std::endl;
    std::cout << "\t Id: " << objectList[objectIndex].id << std::endl;
    std::cout << "\t graspForce: " << objectList[objectIndex].graspForce << std::endl;

    printDoubleVector(objectList[objectIndex].takePosition, std::string("\t Take Position:"));
    printDoubleVector(objectList[objectIndex].takeOrientation, std::string("\t Take Orientation:"));
    printDoubleVector(objectList[objectIndex].placePosition, std::string("\t Place Position:"));
    printDoubleVector(objectList[objectIndex].placeOrientation, std::string("\t Place Orientation:"));

    std::cout << "\t Known object: " << objectList[objectIndex].isKnown << std::endl;

    std::cout << std::endl;
}


void objectsList::printDoubleVector( std::vector<double> vector_,
                                   std::string vectorName )
{
    int listDim = (int)vector_.size(), i = 0;
    std::cout << vectorName << " ";
    for(i = 0; i<listDim; ++i){
        std::cout << vector_[i] << "\t";
    }
    std::cout << std::endl;
}


bool objectsList::saveObjectListToXml(std::string xmlFileName){
    TiXmlElement* objXmlList = new TiXmlElement("ObjectsList");
    objectXmlFile->LinkEndChild(objXmlList);

    int dim_ = (int)objectList.size();
    for(int i=0; i<dim_; ++i){
        //if(objectList[i].isKnown){ // Save only learned objects
            TiXmlElement* object = new TiXmlElement("object");
            objXmlList->LinkEndChild(object);

            // Name
            TiXmlElement* name_ = new TiXmlElement("name");
            object->LinkEndChild(name_);
            name_->SetAttribute("value", objectList[i].name);

            // ID
            TiXmlElement* id_ = new TiXmlElement("id");
            object->LinkEndChild(id_);
            id_->SetAttribute("value", objectList[i].id);

            // trayID
            TiXmlElement* trayId_ = new TiXmlElement("trayId");
            object->LinkEndChild(trayId_);
            trayId_->SetAttribute("value", objectList[i].trayId);

            // Grasp force
            TiXmlElement* force_ = new TiXmlElement("graspForce");
            object->LinkEndChild(force_);
            force_->SetDoubleAttribute("value", objectList[i].graspForce);

            // Pre-Take position
            TiXmlElement* preTakePos = new TiXmlElement("preTakePosition");
            object->LinkEndChild(preTakePos);
            preTakePos->SetDoubleAttribute("x", objectList[i].preTakePosition[0]);
            preTakePos->SetDoubleAttribute("y", objectList[i].preTakePosition[1]);
            preTakePos->SetDoubleAttribute("z", objectList[i].preTakePosition[2]);

            // Pre-Take orientation
            TiXmlElement* preTakeOri = new TiXmlElement("preTakeOrientation");
            object->LinkEndChild(preTakeOri);
            preTakeOri->SetDoubleAttribute("x", objectList[i].preTakeOrientation[0]);
            preTakeOri->SetDoubleAttribute("y", objectList[i].preTakeOrientation[1]);
            preTakeOri->SetDoubleAttribute("z", objectList[i].preTakeOrientation[2]);

            // Pre-Place position
            TiXmlElement* prePlacePos = new TiXmlElement("prePlacePosition");
            object->LinkEndChild(prePlacePos);
            prePlacePos->SetDoubleAttribute("x", objectList[i].prePlacePosition[0]);
            prePlacePos->SetDoubleAttribute("y", objectList[i].prePlacePosition[1]);
            prePlacePos->SetDoubleAttribute("z", objectList[i].prePlacePosition[2]);

            // Pre-Place orientation
            TiXmlElement* prePlaceOri = new TiXmlElement("prePlaceOrientation");
            object->LinkEndChild(prePlaceOri);
            prePlaceOri->SetDoubleAttribute("x", objectList[i].prePlaceOrientation[0]);
            prePlaceOri->SetDoubleAttribute("y", objectList[i].prePlaceOrientation[1]);
            prePlaceOri->SetDoubleAttribute("z", objectList[i].prePlaceOrientation[2]);

            // Take position
            TiXmlElement* takePos = new TiXmlElement("takePosition");
            object->LinkEndChild(takePos);
            takePos->SetDoubleAttribute("x", objectList[i].takePosition[0]);
            takePos->SetDoubleAttribute("y", objectList[i].takePosition[1]);
            takePos->SetDoubleAttribute("z", objectList[i].takePosition[2]);

            // Take orientation
            TiXmlElement* takeOri = new TiXmlElement("takeOrientation");
            object->LinkEndChild(takeOri);
            takeOri->SetDoubleAttribute("x", objectList[i].takeOrientation[0]);
            takeOri->SetDoubleAttribute("y", objectList[i].takeOrientation[1]);
            takeOri->SetDoubleAttribute("z", objectList[i].takeOrientation[2]);

            // Take position
            TiXmlElement* placePos = new TiXmlElement("placePosition");
            object->LinkEndChild(placePos);
            placePos->SetDoubleAttribute("x", objectList[i].placePosition[0]);
            placePos->SetDoubleAttribute("y", objectList[i].placePosition[1]);
            placePos->SetDoubleAttribute("z", objectList[i].placePosition[2]);

            // Take orientation
            TiXmlElement* placeOri = new TiXmlElement("placeOrientation");
            object->LinkEndChild(placeOri);
            placeOri->SetDoubleAttribute("x", objectList[i].placeOrientation[0]);
            placeOri->SetDoubleAttribute("y", objectList[i].placeOrientation[1]);
            placeOri->SetDoubleAttribute("z", objectList[i].placeOrientation[2]);

            // Post-Take position
            TiXmlElement* postTakePos = new TiXmlElement("postTakePosition");
            object->LinkEndChild(postTakePos);
            postTakePos->SetDoubleAttribute("x", objectList[i].postTakePosition[0]);
            postTakePos->SetDoubleAttribute("y", objectList[i].postTakePosition[1]);
            postTakePos->SetDoubleAttribute("z", objectList[i].postTakePosition[2]);

            // Post-Take orientation
            TiXmlElement* postTakeOri = new TiXmlElement("postTakeOrientation");
            object->LinkEndChild(postTakeOri);
            postTakeOri->SetDoubleAttribute("x", objectList[i].postTakeOrientation[0]);
            postTakeOri->SetDoubleAttribute("y", objectList[i].postTakeOrientation[1]);
            postTakeOri->SetDoubleAttribute("z", objectList[i].postTakeOrientation[2]);

            // Post-Place position
            TiXmlElement* postPlacePos = new TiXmlElement("postPlacePosition");
            object->LinkEndChild(postPlacePos);
            postPlacePos->SetDoubleAttribute("x", objectList[i].postPlacePosition[0]);
            postPlacePos->SetDoubleAttribute("y", objectList[i].postPlacePosition[1]);
            postPlacePos->SetDoubleAttribute("z", objectList[i].postPlacePosition[2]);

            // Post-Place orientation
            TiXmlElement* postPlaceOri = new TiXmlElement("postPlaceOrientation");
            object->LinkEndChild(postPlaceOri);
            postPlaceOri->SetDoubleAttribute("x", objectList[i].postPlaceOrientation[0]);
            postPlaceOri->SetDoubleAttribute("y", objectList[i].postPlaceOrientation[1]);
            postPlaceOri->SetDoubleAttribute("z", objectList[i].postPlaceOrientation[2]);

            // Actions
            TiXmlElement* actions = new TiXmlElement("actions");
            object->LinkEndChild(actions);
          /*  TiXmlElement* actNum = new TiXmlElement("numberOfActions");
            actions->LinkEndChild(actNum);
            actNum->SetDoubleAttribute("value", (int)objectList[i].actions_.size());*/

            for(int j=0; j<(int)objectList[i].actions_.size(); ++j){
                // Object actions
                TiXmlElement* action = new TiXmlElement("action");
                actions->LinkEndChild(action);

                TiXmlElement* label_ = new TiXmlElement("label");
                action->LinkEndChild(label_);
                label_->SetAttribute("value", objectList[i].actions_[j].label_);

                TiXmlElement* frame_ = new TiXmlElement("frame");
                action->LinkEndChild(frame_);
                frame_->SetAttribute("value", objectList[i].actions_[j].frame_);

                TiXmlElement* pos_ = new TiXmlElement("position");
                action->LinkEndChild(pos_);
                pos_->SetDoubleAttribute("x", objectList[i].actions_[j].positions_[0][0]);
                pos_->SetDoubleAttribute("y", objectList[i].actions_[j].positions_[0][1]);
                pos_->SetDoubleAttribute("z", objectList[i].actions_[j].positions_[0][2]);

                TiXmlElement* ori_ = new TiXmlElement("orientation");
                action->LinkEndChild(ori_);
                ori_->SetDoubleAttribute("x", objectList[i].actions_[j].orientations_[0][0]);
                ori_->SetDoubleAttribute("y", objectList[i].actions_[j].orientations_[0][1]);
                ori_->SetDoubleAttribute("z", objectList[i].actions_[j].orientations_[0][2]);
            }
       // }
    }

    // Save xml file
    bool fileSaved = objectXmlFile->SaveFile(xmlFileName);
    if(fileSaved){
        return true;
    }
    else{
        return false;
    }

    // Free memory
    objectXmlFile->Clear();
}


std::string objectsList::numToString(int number){
    std::stringstream s;
    s << number;
    return s.str();
}


bool objectsList::createObjectListFromXml(std::string xmlFileName){
    // Load Xml data
    bool fileLoaded = loadObjectListFromXml(xmlFileName);
    if(!fileLoaded){
        std::cout << "XML file not found" << std::endl;
        return fileLoaded;
    }
    // Fill object list
    TiXmlElement *obj    = NULL;
    TiXmlElement *objAtt = NULL;
    TiXmlElement *actions = NULL;
    TiXmlElement *act = NULL;
    TiXmlElement *actAtt = NULL;

    int res_ = 0;

    for(obj = objRootElement->FirstChildElement(); obj != NULL; obj = obj->NextSiblingElement()){
        // Name
        objAtt = obj->FirstChildElement("name");
        newObject.name = std::string(objAtt->Attribute("value"));

        // Id
        objAtt = obj->FirstChildElement("id");
        res_ = objAtt->QueryIntAttribute("value", &newObject.id);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query id returned: " << res_ << std::endl;
            return false;
        }

        // trayId
        objAtt = obj->FirstChildElement("trayId");
        res_ = objAtt->QueryIntAttribute("value", &newObject.trayId);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query id returned: " << res_ << std::endl;
            return false;
        }

        // Grasp force
        objAtt = obj->FirstChildElement("graspForce");
        res_ = objAtt->QueryDoubleAttribute("value", &newObject.graspForce);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query graspForce returned: " << res_ << std::endl;
            return false;
        }
        //std::cout << newObject.graspForce << std::endl;

        // Pre-Take position
        objAtt = obj->FirstChildElement("preTakePosition");
        res_ = objAtt->QueryDoubleAttribute("x", &newObject.preTakePosition[0]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'x' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("y", &newObject.preTakePosition[1]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'y' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("z", &newObject.preTakePosition[2]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'z' returned: " << res_ << std::endl;
            return false;
        }


        // Pre-Take orientation
        objAtt = obj->FirstChildElement("preTakeOrientation");
        res_ = objAtt->QueryDoubleAttribute("x", &newObject.preTakeOrientation[0]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'x' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("y", &newObject.preTakeOrientation[1]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'y' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("z", &newObject.preTakeOrientation[2]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'z' returned: " << res_ << std::endl;
            return false;
        }

        // Pre-Place position
        objAtt = obj->FirstChildElement("prePlacePosition");
        res_ = objAtt->QueryDoubleAttribute("x", &newObject.prePlacePosition[0]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'x' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("y", &newObject.prePlacePosition[1]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'y' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("z", &newObject.prePlacePosition[2]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'z' returned: " << res_ << std::endl;
            return false;
        }

        // Pre-Place orientation
        objAtt = obj->FirstChildElement("prePlaceOrientation");
        res_ = objAtt->QueryDoubleAttribute("x", &newObject.prePlaceOrientation[0]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'x' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("y", &newObject.prePlaceOrientation[1]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'y' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("z", &newObject.prePlaceOrientation[2]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'z' returned: " << res_ << std::endl;
            return false;
        }

        // Take position
        objAtt = obj->FirstChildElement("takePosition");
        res_ = objAtt->QueryDoubleAttribute("x", &newObject.takePosition[0]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'x' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("y", &newObject.takePosition[1]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'y' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("z", &newObject.takePosition[2]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'z' returned: " << res_ << std::endl;
            return false;
        }

        // Take orientation
        objAtt = obj->FirstChildElement("takeOrientation");
        res_ = objAtt->QueryDoubleAttribute("x", &newObject.takeOrientation[0]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takeOrientation 'x' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("y", &newObject.takeOrientation[1]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takeOrientation 'y' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("z", &newObject.takeOrientation[2]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takeOrientation 'z' returned: " << res_ << std::endl;
            return false;
        }
        //std::cout << newObject.takeOrientation[0] << " " << newObject.takeOrientation[1] << " " << newObject.takeOrientation[2] << std::endl;

        // Place position
        objAtt = obj->FirstChildElement("placePosition");
        res_ = objAtt->QueryDoubleAttribute("x", &newObject.placePosition[0]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query placePosition 'x' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("y", &newObject.placePosition[1]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query placePosition 'y' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("z", &newObject.placePosition[2]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query placePosition 'z' returned: " << res_ << std::endl;
            return false;
        }
        //std::cout << newObject.placePosition[0] << " " << newObject.placePosition[1] << " " << newObject.placePosition[2] << std::endl;

        // Place orientation
        objAtt = obj->FirstChildElement("placeOrientation");
        res_ = objAtt->QueryDoubleAttribute("x", &newObject.placeOrientation[0]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query placeOrientation 'x' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("y", &newObject.placeOrientation[1]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query placeOrientation 'y' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("z", &newObject.placeOrientation[2]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query placeOrientation 'z' returned: " << res_ << std::endl;
            return false;
        }
        //std::cout << newObject.placeOrientation[0] << " " << newObject.placeOrientation[1] << " " << newObject.placeOrientation[2] << std::endl;

        // Post-Take position
        objAtt = obj->FirstChildElement("postTakePosition");
        res_ = objAtt->QueryDoubleAttribute("x", &newObject.postTakePosition[0]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'x' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("y", &newObject.postTakePosition[1]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'y' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("z", &newObject.postTakePosition[2]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'z' returned: " << res_ << std::endl;
            return false;
        }

        // Post-Take orientation
        objAtt = obj->FirstChildElement("postTakeOrientation");
        res_ = objAtt->QueryDoubleAttribute("x", &newObject.postTakeOrientation[0]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'x' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("y", &newObject.postTakeOrientation[1]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'y' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("z", &newObject.postTakeOrientation[2]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'z' returned: " << res_ << std::endl;
            return false;
        }

        // Post-Place position
        objAtt = obj->FirstChildElement("postPlacePosition");
        res_ = objAtt->QueryDoubleAttribute("x", &newObject.postPlacePosition[0]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'x' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("y", &newObject.postPlacePosition[1]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'y' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("z", &newObject.postPlacePosition[2]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'z' returned: " << res_ << std::endl;
            return false;
        }

        // Post-Place orientation
        objAtt = obj->FirstChildElement("postPlaceOrientation");
        res_ = objAtt->QueryDoubleAttribute("x", &newObject.postPlaceOrientation[0]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'x' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("y", &newObject.postPlaceOrientation[1]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'y' returned: " << res_ << std::endl;
            return false;
        }
        res_ = objAtt->QueryDoubleAttribute("z", &newObject.postPlaceOrientation[2]);
        if(res_ != TIXML_SUCCESS){
            std::cout << "Query takePosition 'z' returned: " << res_ << std::endl;
            return false;
        }

        newObject.isKnown = false;

        // Load actions
        actions = obj->FirstChildElement("actions");
        if(actions!=NULL){
            // Init action struct
            std::vector<double> currPt(3, 0.0);
            objectsList::actionDescriptor newAction;

            newAction.positions_.push_back(currPt);
            newAction.orientations_.push_back(currPt);

            for(act = actions->FirstChildElement(); act != NULL; act = act->NextSiblingElement()){
                actAtt = act->FirstChildElement("label");
                newAction.label_ = std::string(actAtt->Attribute("value"));

                actAtt = act->FirstChildElement("frame");
                newAction.frame_ = std::string(actAtt->Attribute("value"));

                // Position
                actAtt = act->FirstChildElement("position");
                res_ = actAtt->QueryDoubleAttribute("x", &newAction.positions_[0][0]);
                if(res_ != TIXML_SUCCESS){
                    std::cout << "Query position 'x' returned: " << res_ << std::endl;
                    return false;
                }
                res_ = actAtt->QueryDoubleAttribute("y", &newAction.positions_[0][1]);
                if(res_ != TIXML_SUCCESS){
                    std::cout << "Query position 'y' returned: " << res_ << std::endl;
                    return false;
                }
                res_ = actAtt->QueryDoubleAttribute("z", &newAction.positions_[0][2]);
                if(res_ != TIXML_SUCCESS){
                    std::cout << "Query position 'z' returned: " << res_ << std::endl;
                    return false;
                }

               // std::cout << newAction.positions_[0][0] << " " << newAction.positions_[0][1] << " " << newAction.positions_[0][2] << std::endl;


                // Orientation
                actAtt = act->FirstChildElement("orientation");
                res_ = actAtt->QueryDoubleAttribute("x", &newAction.orientations_[0][0]);
                if(res_ != TIXML_SUCCESS){
                    std::cout << "Query position 'x' returned: " << res_ << std::endl;
                    return false;
                }
                res_ = actAtt->QueryDoubleAttribute("y", &newAction.orientations_[0][1]);
                if(res_ != TIXML_SUCCESS){
                    std::cout << "Query position 'y' returned: " << res_ << std::endl;
                    return false;
                }
                res_ = actAtt->QueryDoubleAttribute("z", &newAction.orientations_[0][2]);
                if(res_ != TIXML_SUCCESS){
                    std::cout << "Query position 'z' returned: " << res_ << std::endl;
                    return false;
                }

               // std::cout << newAction.orientations_[0][0] << " " << newAction.orientations_[0][1] << " " << newAction.orientations_[0][2] << std::endl;


                newObject.actions_.push_back(newAction);

            }
        }





        // Add object to the list
        updateObjectList();
    }

    // Free memory
    objectXmlFile->Clear();

    return fileLoaded;
}
