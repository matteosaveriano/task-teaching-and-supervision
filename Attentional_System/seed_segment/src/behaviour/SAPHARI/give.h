#include "../../seed_header.h"
#include "saphari.h"


#ifndef _GIVE_B_PR2_
#define _GIVE_B_PR2_


class giveBehaviour : public pr2Behaviour {
	public:
		giveBehaviour(std::string instance){
		setName(instance2vector(instance)[0]);
		setInstance(instance);
		setRtm(QUIESCENCE);
		target=instance2vector(instance)[1];
		agent=instance2vector(instance)[2];
		
		
		std::cout << "Give class: " << " agent: " << agent << " target: " << target << std::endl;
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

		this->updateRtm(WMV.get<double>("dist(pr2," + agent + ")"), GAIN_DIST_MAX, GAIN_DIST_MIN);
		
		//se la mano di pr2 ha il target 
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
		ss<<"( AttentionalInterface.execute handover (. PR2_ROBOT HERAKLES_HUMAN1 "<<SEED2laas_param(target)<< "  .)"<<mpSender<< " )";
		return ss.str();
	}
	
	void update_wmv() {
		
		std::stringstream ss;
		ss << agent << "." << target;

		std::cout << "update_wmv: " << ss.str() << std::endl;

		WMV.set<double>(ss.str(), 1);
		
		ss.str("");
		ss << "hand" << "." << target;
		WMV.set<double>(ss.str(), 0);

		WMV.set< std::string >("pr2.hand", "null");
		WMV.set< double >("pr2.handFree", 1);
	
	}
};

#endif