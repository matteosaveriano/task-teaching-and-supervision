/* 
 * File:   mie_Forget.h
 * Author: hargalaten
 *
 * Created on 16 dicembre 2012, 12.41
 */

#ifndef MIE_FORGET_H
#define	MIE_FORGET_H

#include "../seed_header.h"

class ForgetBehaviour : public Behaviour{
public:
    ForgetBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
    }
    bool perceptualSchema(){
        return true;
    }
    void motorSchema(){
        std::vector <WM_node *> remNode;
        pthread_mutex_lock(&memMutex);
        
        if(WM==NULL){
            pthread_mutex_unlock(&memMutex);
            return;
        }
        
        remNode=WM->getNodesByInstance(instance2vector(this->getInstance())[1]);
        std::cout<<"removing: "<<instance2vector(this->getInstance())[1]<<"\n";
        for(int i=0;i<remNode.size();i++)
        {
            remove(remNode[i]);
            if(remNode[i]==WM){
                WM=NULL;
                pthread_mutex_unlock(&memMutex);
                return;
            }
        }
        remove(WM->getNodesByInstance(this->getInstance())[0]);
        
        pthread_mutex_unlock(&memMutex);
    }
    void exit(){};
    void start(){};
};


#endif	/* MIE_FORGET_H */

