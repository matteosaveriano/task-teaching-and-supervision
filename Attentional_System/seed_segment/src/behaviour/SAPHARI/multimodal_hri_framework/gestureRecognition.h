/* 
 * File:   gestureRecognition.h
 * Author: hargalaten
 *
 * Created on 8 settembre 2014, 16.10
 */

#ifndef GESTURERECOGNITION_H
#define	GESTURERECOGNITION_H

#include "../../../seed_header.h"
#include "ros/callback_queue.h"
#include "seed_segment/Result.h"

#define GESTURENUMBER 8

std::pair<std::string,double> command[GESTURENUMBER];
std::string pointedObject;
bool foundGesture=false;
bool foundPointing=false;

// All'arrivo del gesto comincia a riempire il vettore
void gestureCallback(const seed_segment::Result::ConstPtr& msg)
{
//    std::cout<<"Gesture Result Callback start!\n";
	for(int i=0; i<GESTURENUMBER; i++)
	{   
            command[i].first = msg->resultData[i].numClass;
            
//                std::cout<<"Gesture"<<i<<"\n";
            command[i].second = msg->resultData[i].probability;
	}
        foundGesture=true;
	//std::cout<<"Gesture Result Callback: "<<command[0].first<<std::endl;
}

// Oggetto a cui è rivolto il braccio
void pointingCallback(const std_msgs::String::ConstPtr& msg)
{
    pointedObject=msg->data;
    foundPointing=true;
    
}

class GestureRecognitionBehaviour : public Behaviour{
public:
    GestureRecognitionBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        rGesture= nh.subscribe("Gesture_Result", 0, gestureCallback);
        pObject= nh2.subscribe("Point_Object", 0, pointingCallback);
        
        
     }
    bool perceptualSchema(){
        ros::spinOnce();
        
        pthread_mutex_lock(&memMutex);
        this->setRtm(0.1);
        pthread_mutex_unlock(&memMutex);
        
        return foundGesture;
    }
    void motorSchema(){
        pthread_mutex_lock(&memMutex);
        
        WM_node* me=WM->getNodesByInstance(this->getInstance())[0];
        while(me->son.size()!=0){
            remove(me->son[0]);
        }
        
        for(int i=0;i<GESTURENUMBER;i++){
            std::stringstream ss;
            ss<<"gesture("<<command[i].first<<")";
            WM_node* son = me->addSon(ss.str());
            son->amplification = command[i].second;
            
            if(command[i].first=="point" && foundPointing){
                std::stringstream ssp;
                ssp<<"object("<<pointedObject<<")";
                //WM_node* obj = son->addSon(ssp.str());
                //obj->amplification = command[i].second;
                foundPointing=false;
            }
        }
        pthread_mutex_unlock(&memMutex);
        
        foundGesture=false;
        }
    void start(){
        std::cout<<"Alfred: gestureRecognition on!\n";
    }
    void exit(){
        std::cout<<"Alfred: gestureRecognition off!\n";
    }
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh2;
    ros::Subscriber rGesture;
    ros::Subscriber pObject;
    
};

#endif	/* GESTURERECOGNITION_H */

