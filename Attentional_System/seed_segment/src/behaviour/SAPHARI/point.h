#include "../../seed_header.h"
#include "saphari.h"


#ifndef _POINT_B_PR2_
#define _POINT_B_PR2_



class pointBehaviour : public pr2Behaviour{

public:
    pointBehaviour(std::string instance){
      setName(instance2vector(instance)[0]);
      setInstance(instance);
      setRtm(QUIESCENCE);
      position=instance2vector(instance)[1];

      if( !SIMULATE_HATP ) {

        mpSender = instance;
        mpSender.erase(std::remove(mpSender.begin(), mpSender.end(), ','), mpSender.end());
        mpSender.erase(std::remove(mpSender.begin(), mpSender.end(), '('), mpSender.end());
        mpSender.erase(std::remove(mpSender.begin(), mpSender.end(), ')'), mpSender.end());
        
        std::cout<<"registring "<<mpSender<<" to OPRS\n";
        mpSocket=external_register_to_the_mp_host_prot(mpSender.c_str(),host_name.c_str(),3300,STRINGS_PT);
      }
    }
    bool perceptualSchema(){
      bool release;
      pthread_mutex_lock(&memMutex);
                    
      //se la mano di pr2 è vuota 
      if(WMV.get< std::string >("pr2.hand") == "null" )
       //attiva lo schema motorio
       release=true;
       //altrimenti
      else
        //non attivare lo schema motorio
        release=false;
        
      pthread_mutex_unlock(&memMutex);
      return release;
    }
    std::string oprs_message(){
        std::stringstream ss;
        ss<<"(AttentionalInterface.execute pointTo (. "<<SEED2laas_param(position)<< " .) "<<mpSender<< " )";
        return ss.str();
    }
     void update_wmv() {
    
      std::stringstream ss;
      ss << "pointTo(" << position << ").done";
      WMV.set<double>(ss.str(), 1);
      
      }
    void exit() {
      std::stringstream ss;
      ss << "pointTo(" << position << ").done";
      WMV.set<double>(ss.str(), 0);      
    }
};

#endif