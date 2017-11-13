#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <rosgraph_msgs/Clock.h>
#include <sys/time.h>

#include "kuka_seed_commands/objectsList.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

std::string numToString(int number){
    std::stringstream s;
    s << number;
    return s.str();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "objectsSimulator");

    ros::NodeHandle node;

    bool pubEE = false;
    node.getParam("publish_end_effector", pubEE);

    ros::Publisher arPub = node.advertise<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 0);

    ros::Publisher tfPub = node.advertise<tf::tfMessage>("/tf", 0);

    std::string fileName = ros::package::getPath("kuka_seed_commands")
                            + "/data/objects_database_coffee.xml";
    
    objectsList *ol = new objectsList(true, fileName);

    tf::TransformListener listener;

    std::string worldFrame = "/camera_rgb_optical_frame";

    // Init tf message
    tf::tfMessage tfMsg;
    tfMsg.transforms.resize(ol->objectList.size());

    // Init ar and tf messages
    ar_track_alvar_msgs::AlvarMarkers arMsg;
    ar_track_alvar_msgs::AlvarMarker tmpMsg;

    size_t i = 0;
    size_t dim_ = ol->objectList.size();
    for(i=0; i<dim_; ++i){
        tmpMsg.id = ol->objectList[i].id;
        arMsg.markers.push_back(tmpMsg);

        tfMsg.transforms[i].child_frame_id = "/ar_marker_"+numToString(ol->objectList[i].id);
        tfMsg.transforms[i].header.frame_id = worldFrame;
        tfMsg.transforms[i].header.stamp = ros::Time::now();
        tfMsg.transforms[i].transform.translation.x = 0.0;
        tfMsg.transforms[i].transform.translation.y = 0.0;
        tfMsg.transforms[i].transform.translation.z = 0.0;
        tfMsg.transforms[i].transform.rotation.w = 1.0;
        tfMsg.transforms[i].transform.rotation.x = 0.0;
        tfMsg.transforms[i].transform.rotation.y = 0.0;
        tfMsg.transforms[i].transform.rotation.z = 0.0; 
    }

    tf::tfMessage eeMsg;
    if(pubEE){
        eeMsg.transforms.resize(1);

        eeMsg.transforms[0].child_frame_id = "/wsg50_end_link";
        eeMsg.transforms[0].header.frame_id = "/world";

        eeMsg.transforms[0].transform.translation.x = -0.7353;
        eeMsg.transforms[0].transform.translation.y = -0.072909;
        eeMsg.transforms[0].transform.translation.z = 0.40786;
        eeMsg.transforms[0].transform.rotation.w = 0.237;
        eeMsg.transforms[0].transform.rotation.x = 0.66952;
        eeMsg.transforms[0].transform.rotation.y = -0.31207;
        eeMsg.transforms[0].transform.rotation.z = -0.63102;
    }

    std::cout << "\n Object simulator started...\n";

	sleep(2);
    size_t iter = 0;
    int loopCnt = 0;
    while(ros::ok()){
        tf::StampedTransform transform;
        ros::Time currTime = ros::Time::now();
        // Listen all objects
        for(i=0; i<dim_; ++i){
            tfMsg.transforms[i].header.stamp = currTime;
        	try{
                listener.lookupTransform( worldFrame,
                                          "/"+ol->objectList[i].name,
                                          ros::Time(0),
                                          transform );
                // Convert to geometry message
                tf::Pose tfPose(transform.getBasis(), transform.getOrigin());
                tf::Stamped<tf::Pose> tfTranPose(tfPose, transform.stamp_, transform.frame_id_);
                tf::poseStampedTFToMsg(tfTranPose, arMsg.markers[i].pose);    
                // Convert to tf message

                tfMsg.transforms[i].header.stamp = ros::Time::now();
                tfMsg.transforms[i].transform.translation.x = transform.getOrigin().getX();
                tfMsg.transforms[i].transform.translation.y = transform.getOrigin().getY();
                tfMsg.transforms[i].transform.translation.z = transform.getOrigin().getZ();
                tfMsg.transforms[i].transform.rotation.w = transform.getRotation().getW();
                tfMsg.transforms[i].transform.rotation.x = transform.getRotation().getX();
                tfMsg.transforms[i].transform.rotation.y = transform.getRotation().getY();
                tfMsg.transforms[i].transform.rotation.z = transform.getRotation().getZ();
            }
            catch (tf::TransformException ex){
                if(iter>dim_)
                    ROS_WARN("%s",ex.what());

				iter++;
            }
        }
        // Publish ar message
        arPub.publish(arMsg);
        // Publish tf message
        tfPub.publish(tfMsg);

        if(pubEE && loopCnt++){
            tfPub.publish(eeMsg);
        }

        // Sleep 
        usleep(30000);//loopRate.sleep();
    }

    delete ol;
    return 0;
}
