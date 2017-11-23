/* 
 * File:   mie_Listing.h
 * Author: hargalaten
 *
 * Created on 17 dicembre 2012, 18.34
 */

#ifndef MIE_LISTING_H
#define	MIE_LISTING_H

#include "../seed_header.h"

class ListingBehaviour : public Behaviour{
public:
    ListingBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
    }
    bool perceptualSchema(){
        return true;
    }
    void motorSchema(){
        pthread_mutex_lock(&memMutex);
        if(dead()) return;
        remove((WM->getNodesByInstance(this->getInstance()))[0]);
        printWM(WM);
        pthread_mutex_unlock(&memMutex);
    }
    void run(){
        
    }
    void start(){}
    void exit(){}
};

#endif	/* MIE_LISTING_H */

