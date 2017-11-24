#include "../../seed_header.h"
#include "saphari.h"


#ifndef _GO_B_PR2_
#define _GO_B_PR2_

class goBehaviour : public pr2Behaviour{
	public:
    goBehaviour(std::string instance){
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
        
      this->updateRtm(WMV.get<double>("dist(pr2," + position + ")"), GAIN_DIST_MAX, GAIN_DIST_MIN);
      
      release = true;

      pthread_mutex_unlock(&memMutex);
      return release;
	  }
  
    std::string oprs_message(){
     	std::stringstream ss;
      ss<<"( AttentionalInterface.execute go (. "<<SEED2laas_param(position)<< " .)"<<mpSender<< " )";
      return ss.str();
    }

    void update_wmv() {
    
      std::stringstream ss;
      ss << "go(" << position << ").arrived";
      WMV.set<double>(ss.str(), 1);
      
      }
    void exit() {
      close(mpSocket);
      std::stringstream ss;
      ss << "go(" << position << ").arrived";
      WMV.set<double>(ss.str(), 0);      
    }
};

#endif