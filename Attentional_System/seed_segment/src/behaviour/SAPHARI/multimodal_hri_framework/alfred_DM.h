/* 
 * File:   Alfred_DM.h
 * Author: hargalaten
 *
 * Created on 10 aprile 2013, 10.26
 */

#ifndef ALFRED_DM_H
#define	ALFRED_DM_H

#include "mie_header.h"
//#include "AlfredDialogueManager/POMDP.h"
#include "POMDP.h"

class AlfredDMBehaviour : public Behaviour{
public:
    AlfredDMBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        char **app=new char*[2];
//        char **app =(char**)malloc (sizeof(char*)* 2);
        std::string str1="AlfredDialogueManager/No.xml";
        std::string str2="AlfredDialogueManager/Cane_Riporto.xml";
        app[0]= new char[str1.length()+1];
        app[1]= new char[str2.length()+1];
        strcpy(app[0],str1.c_str());
        strcpy(app[1],str2.c_str());
        
        pomdp=new POMDP(2,app);
        delete [] app;
    }
    bool perceptualSchema(){
        bool obs;
        pthread_mutex_lock(&memMutex);
        WMV.get("HAI.observed",&obs);
        WMV.set("HAI.observed",false);
        
        if(dead()) return false;
        
        WM_node *me=WM->getNodesByInstance(getInstance())[0];
        if(!me->son.empty() && me->son[0]->goalStatus()){
            me->amplification=0;
            done=true;
            remove(me->son[0]);
        }
        pthread_mutex_unlock(&memMutex);
        
        return obs;
        
    }
    void motorSchema(){
        std::string action;
        pomdp->updateBelieves(NBestObs);
        action=pomdp->performAction("greedy");
        std::cout<<"ALFRED: "<<action<<"\n";

        
    }
    void start(){
        std::cout<<"ALFRED: hi, how can i help you?\n";
    }
    void exit(){
        
    }
protected:
    POMDP *pomdp;
    vector<Observ> NBestObs;
    bool done;
};


#endif	/* ALFRED_DM_H */

