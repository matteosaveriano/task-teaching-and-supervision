#include "../../seed_header.h"
#include "saphari.h"


#ifndef _ATTACH_BR_B_PR2_
#define _ATTACH_BR_B_PR2_

class attach_bracketBehaviour : public pr2Behaviour{
	public:
    attach_bracketBehaviour(std::string instance){
      setName(instance2vector(instance)[0]);
      setInstance(instance);
      setRtm(QUIESCENCE);
      target=instance2vector(instance)[1];
      position=instance2vector(instance)[2];
      
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
      //se la mano di pr2 è vuota 
      if(WMV.get< std::string >("pr2.hand") == target )
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
      ss<<"( AttentionalInterface.execute attachBracket (. "<<SEED2laas_param(target)<<" "<<SEED2laas_param(position)<< " .) "<<mpSender<< " )";
      return ss.str();
    }
    void update_wmv() {

      std::stringstream ss;
      ss << position << ".installed";
      WMV.set<double>(ss.str(), 1);
      
      ss.str("");
      WMV.set< std::string >("pr2.hand", "null");
      WMV.set< double >("hand.free", 1);

      ss.str("");
      ss << "hand." << target;
      WMV.set<double>(ss.str(), 0);
    }
};

#endif