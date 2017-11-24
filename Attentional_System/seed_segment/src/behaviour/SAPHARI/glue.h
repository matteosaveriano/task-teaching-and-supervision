#include "../../seed_header.h"
#include "saphari.h"


#ifndef _GLUE_B_PR2_
#define _GLUE_B_PR2_

class glueBehaviour : public pr2Behaviour{
	public:
    glueBehaviour(std::string instance){
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
        
      this->updateRtm(2,3,1);
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
      ss<<"( AttentionalInterface.execute glue (. GLUE_BOTTLE "<<SEED2laas_param(position)<< " )."<<mpSender<< " )";
      return ss.str();
    }
    void update_wmv() {
      
      std::stringstream ss;
      ss << position << ".pasted";
      WMV.set<double>(ss.str(), 1);
    
    }
};

#endif