#include "../seed_header.h"


#ifndef INPUT_STREAM_H
#define INPUT_STERAM_H

class IOBehaviour : public Behaviour{
public:
    void exit(){
        //std::cout<<getName()<<" EXITED\n";
    }
    void start(){
        std::cout<<"SYSTEM: "<<getName()<<" on!\n";
    };
};
  

class InputStreamBehaviour : public IOBehaviour {
public:   
    InputStreamBehaviour(std::string instance){

        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        reqStream="requestStream";

        
    }
    bool perceptualSchema(){
        //std::cin>>input;
        
        bool new_input = false;

        std::getline(std::cin,input);
        importance=0;
        while(input[input.size()-1]=='!'){
            input.erase(input.size()-1);
            importance++;
            
        }

        return true;
    }
    void motorSchema(){
        WM_node *son;
        WM_node *req;
        
        std::cout<< "SYSTEM: " << input << "\n";
        
        pthread_mutex_lock(&memMutex);
        if(WM!=NULL)
            if((req=WM->getNodesByInstance(reqStream)[0]) != NULL){
                son=req->addSon(input);
                son->ltMagnitude+=importance;
            }
            else
                std::cout<<"SYSTEM: no requestStream!\n";
        pthread_mutex_unlock(&memMutex);
        
    }
protected:
    std::string input;
    int importance;
    std::string reqStream;

    ros::NodeHandle nh;
    ros::Subscriber goal_sub;
    std::string goal;
    bool new_goal;
};


class RequestStreamBehaviour : public Behaviour{
public:
    RequestStreamBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        reqNode=WM->getNodesByInstance(instance)[0];
    }
    bool perceptualSchema(){
        return true;
    }
    void motorSchema(){
        pthread_mutex_lock(&memMutex);
        setRtm(DEFAULT);
        for(int i=0;i<reqNode->son.size();i++)
            if(reqNode->son[i]->teleological && 
                    reqNode->son[i]->goalStatus())
                remove(reqNode->son[i]);
        pthread_mutex_unlock(&memMutex);
    }
    void start(){
    }
    void exit(){
    }
private:
    WM_node *reqNode;
        
};

std::string ros_input;

void ros_input_callback(const std_msgs::String::ConstPtr& msg){
    ros_input = msg->data;
}

class RosStreamBehaviour : public IOBehaviour{
public:   
    RosStreamBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        
        sb=nh.subscribe("seed_stream", 0, ros_input_callback);
        
    }
    bool perceptualSchema(){
        //std::cin>>input;
        ros::spinOnce();
        
        if(ros_input == "")
            return false;
        
        importance=0;
        while(ros_input[ros_input.size()-1]=='!'){
            ros_input.erase(ros_input.size()-1);
            importance++;
        }
        return true;
    }
    void motorSchema(){
        WM_node *son;
        WM_node *req;
        
        //std::cout<< "SYSTEM: " << input << "\n";
        
        pthread_mutex_lock(&memMutex);
        if(WM!=NULL){
            if((req=WM->getNodesByInstance( this->getInstance() )[0]) != NULL){
                
                if(req->son.size()!=0)
                    remove(req->son[0]);
                
                son=req->addSon(ros_input);
                son->ltMagnitude+=importance;
            }
            else
                std::cout<<"SYSTEM: no requestStream!\n";
        }
        pthread_mutex_unlock(&memMutex);
        ros_input="";
        
    }
protected:
    int importance;
    ros::NodeHandle nh;
    ros::Subscriber sb;
};


#endif
