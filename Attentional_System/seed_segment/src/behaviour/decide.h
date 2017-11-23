/* 
 * File:   mie_decide.h
 * Author: hargalaten
 *
 * Created on 4 luglio 2013, 15.28
 */

#ifndef MIE_DECIDE_H
#define	MIE_DECIDE_H

#include "../seed_header.h"

#define UNDECISION 0.5

class DecideBehaviour : public Behaviour{
public:
    DecideBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        
        cont1=instance2vector(instance)[1];
        cont2=instance2vector(instance)[2];
        freezed="";
    }
    bool perceptualSchema(){
        bool result=false;
        pthread_mutex_lock(&memMutex);
        
        if(WM->getNodesByInstance(cont1).size()==0 ||
                WM->getNodesByInstance(cont1)[0]->goalStatus() ||
                WM->getNodesByInstance(cont2).size()==0 ||
                WM->getNodesByInstance(cont2)[0]->goalStatus())
            remove(WM->getNodesByInstance(this->getInstance())[0]);
        else 
            result=true;
        
        pthread_mutex_unlock(&memMutex);
        
        return result;
    }
    void motorSchema(){
    
        pthread_mutex_lock(&memMutex);
        
        if(freezed==""){
            //se non ho figli
            if(WM->getNodesByInstance(cont1)[0]->rtm < WM->getNodesByInstance(cont2)[0]->rtm){
                std::cout<<"freezing "<<cont2<<"\n";
                freezed=cont2;
                WM->getNodesByInstance(this->getInstance())[0]->addSon("freeze("+cont2+")");
            }
            else{
                WM->getNodesByInstance(this->getInstance())[0]->addSon("freeze("+cont1+")");
                freezed=cont1;
                std::cout<<"freezing "<<cont1<<"\n";
            }         
        }
//        else {
//            if(WM->getNodesByInstance(cont1)[0]->rtm > WM->getNodesByInstance(cont2)[0]->rtm
//                    && freezed==cont2){
//                remove(WM->getNodesByInstance(this->getInstance())[0]->son[0]);
//                WM->getNodesByInstance(this->getInstance())[0]->addSon("freeze("+cont1+")");
//                freezed=cont1;
//                std::cout<<"freezing "<<cont1<<"\n";
//            }
//            else if(WM->getNodesByInstance(cont1)[0]->rtm < WM->getNodesByInstance(cont2)[0]->rtm
//                    && freezed==cont1){
//                remove(WM->getNodesByInstance(this->getInstance())[0]->son[0]);
//                WM->getNodesByInstance(this->getInstance())[0]->addSon("freeze("+cont1+")");
//                freezed=cont1;
//                std::cout<<"freezing "<<cont1<<"\n";
//            }
//        }
        
        pthread_mutex_unlock(&memMutex);
    }
    void exit(){
        
        std::cout<<this->getInstance()<<" EXITED\n";
    }
    void start(){};
private:
    std::string cont1,cont2,freezed;    
}; 


#endif	/* MIE_DECIDE_H */

