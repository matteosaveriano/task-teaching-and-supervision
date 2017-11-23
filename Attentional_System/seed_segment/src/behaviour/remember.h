/* 
 * File:   mie_Remember.h
 * Author: hargalaten
 *
 * Created on 16 dicembre 2012, 14.36
 */

#ifndef MIE_REMEMBER_H
#define	MIE_REMEMBER_H

#include "../seed_header.h"

class RememberBehaviour : public Behaviour{
public:
    RememberBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
    }
    bool perceptualSchema(){
        return true;
    }
    void motorSchema(){
        
        std::string rememberInstance=instance2vector(this->getInstance())[1];
        pthread_mutex_lock(&memMutex);
        //se la WM è stata deallocata esci
        if(WM==NULL){
            pthread_mutex_unlock(&memMutex);
            return;
        }
        //se l'instanza da ricordare non è presente nella WM
        if((WM->getNodesByInstance(rememberInstance)).size()==0)
            //aggiungi il nuovo nodo alla WM
            WM->addSon(rememberInstance);
        //rimuovi il mio nodo dalla WM
        remove((WM->getNodesByInstance(this->getInstance()))[0]);
        pthread_mutex_unlock(&memMutex);
    }
    void exit(){};
    void start(){};
};


#endif	/* MIE_REMEMBER_H */

