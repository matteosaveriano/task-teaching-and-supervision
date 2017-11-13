#ifndef OBJECTSLIST_H
#define OBJECTSLIST_H

#include <iostream>
#include <vector>
#include <cstdarg>
#include <sstream>
#include <string>
#include "tinyxml.h"

#include <geometry_msgs/PoseStamped.h>

class objectsList
{
public:

    typedef struct {
        std::string label_;
        std::string frame_;
        std::vector< std::vector<double> > positions_;     // Related to frame_ (if size = 1 -> goal)
        std::vector< std::vector<double> > orientations_;  // Related to frame_ (if size = 1 -> goal)
    }actionDescriptor;

    // Object basic structure
    typedef struct {
        std::string name;
        int id;
        int trayId;
        double graspForce; // N (two finger gripper)
        std::vector<double> preTakePosition;     // Related to the QR code (object) 3D position
        std::vector<double> preTakeOrientation;  // Related to the QR code (object) 3D orientation (Euler angles xyz)
        std::vector<double> prePlacePosition;    // Related to the QR code (tray) 3D position
        std::vector<double> prePlaceOrientation; // Related to the QR code (tray) 3D orientation (Euler angles xyz)
        std::vector<double> takePosition;     // Related to the QR code (object) 3D position
        std::vector<double> takeOrientation;  // Related to the QR code (object) 3D orientation (Euler angles xyz)
        std::vector<double> placePosition;    // Related to the QR code (tray) 3D position
        std::vector<double> placeOrientation; // Related to the QR code (tray) 3D orientation (Euler angles xyz)
        std::vector<double> postTakePosition;     // Related to the QR code (object) 3D position
        std::vector<double> postTakeOrientation;  // Related to the QR code (object) 3D orientation (Euler angles xyz)
        std::vector<double> postPlacePosition;    // Related to the QR code (tray) 3D position
        std::vector<double> postPlaceOrientation; // Related to the QR code (tray) 3D orientation (Euler angles xyz)
        bool isKnown;
        geometry_msgs::Pose poseWorldFrame;
        // All the actions associated with the object
        std::vector<objectsList::actionDescriptor> actions_;
    } objectsDescription;

    objectsList( bool loadObjectXmlList  = false,
                 std::string xmlFileName = "" );

    ~objectsList();

    /**** Functions ****/
    // Save and load objects list
    bool saveObjectListToXml(std::string xmlFileName);
    bool loadObjectListFromXml(std::string xmlFileName);

    // Create object list from xml file
    // return false if the file is not found
    bool createObjectListFromXml(std::string xmlFileName);

    // Update object list
    inline void updateObjectList(){objectList.push_back(newObject);}
    inline void updateObjectList(objectsList::objectsDescription object){objectList.push_back(object);}

    // Find an object index in the list using id
    // return true if the object is in the list, false otherwise
    bool findObjectById(int objectID, int &objectIndex);

    // Find an object index in the list using name
    // return true if the object is in the list, false otherwise
    bool findObjectByName(std::string objectName, int &objectIndex);

    // Access an object in the list given its index
    inline objectsList::objectsDescription getObjectByIndex(int objectIndex){return objectList[objectIndex];}

    // Access an object in the list given its ID
    // return true if the object is in the list, false otherwise
    bool getObjectByID(int objectID, objectsList::objectsDescription *object);

    void initObjectStructure(objectsList::objectsDescription *object);

    void printObjectList();
    void printObjectInList(int objectIndex);

    std::string numToString(int number);

    /**** Members ****/
    // Mantain a list of the objects to manipulate
    std::vector<objectsList::objectsDescription> objectList;
    objectsList::objectsDescription newObject;

private:
    /**** Functions ****/
    void printDoubleVector(std::vector<double> vector_, std::string vectorName);

    // Xml file read and write
    TiXmlDocument *objectXmlFile;
    TiXmlElement  *objRootElement;

    bool loadXmlList;
};

#endif // objectSLIST_H
