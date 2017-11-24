#include "../../seed_header.h"
#include "opaque-pub.h"
#include "mp-pub.h"
#include "boost/thread.hpp"
#include <boost/algorithm/string.hpp>    
#include "saphari.h"

#ifndef _PLAN_LISTENER_B_PR2_
#define _PLAN_LISTENER_B_PR2_





std::string after_space(char *message, int *i) {

    std::string ret;
    (*i) = 0;

    while (message[*i]!=' ' ){        
        (*i)++;
    }
    (*i)++;

    while (message[*i]!='\0' ){
      ret += message[*i];
      (*i)++;
    }     


    ret = ret.substr( 0, ret.size()-1);

    return ret;
}




class PlanListenerBehaviour : public IOBehaviour {

	public:
    std::vector< std::string > hatp2vector(std::string fromHATP) {
      bool ended = false;
      char c;
      std::string app;
      std::vector<std::string> result;
      std::stringstream ss(fromHATP);
      int count = 0;
      //non saltare gli spazi!
      ss >> std::noskipws;
      //leggi il primo carattere della stringa
      ss>>c;
      //mentre non sei a fine stringa
      while (!ss.eof() && !ended) {
          if(c == '(' && count == 0){
              ss>>c;
              if(c == '.'){
                  //scarta lo spazio
                  ss>>c;
                  count++;
                  //prendi il prossimo
                  ss>>c;
              }
          }
          else if(c == '(' && count != 0){
              app=app+c;
              //prendi il prossimo
              ss>>c;
              if(c == '.'){
                  //prendi il punto
                  app=app+c;
                  //prendi lo spazio
                  ss>>c;
                  app=app+c;
                  count++;
                  //prendi il prossimo
                  ss>>c;
              }
          }
          else if(c==' ' && count != 1){
              //salva lo spazio
              app=app+c;
              //prendi il prossimo
              ss>>c;
              if(c == '.') {
                  //salva il punto
                  app=app+c;
                  //prendi il prossimo
                  ss>>c;
                  if(c == ')'){
                      //salva la parentesi
                      app=app+c;
                      count--;
                      //prendi il prossimo
                      ss>>c;
                  }
              }
          }
          else if(c==' ' && count == 1){
              //inserisci la stringa d'appoggio nel vettore risultato
              result.push_back(app);
              //pulisci la stringa d'appoggio
              app = "";
              ss>>c;
              if(c == '.'){
                  app=app+c;
                  ss>>c;
                  if(c==')'){
                      count--;
                      ended=true;
                  }
              }
          }
          //se è un punto allora ho trovato una lista vuota
          else if(c == '.'){
              count--;
              //salva il punto
              app=app+c;
              //salva la parentesi chiusa
              ss>>c;
              app=app+c;
              //prendi il prossimo
              ss>>c;
              
          }
          else{
              app=app+c;
              ss>>c;
          }
      }
      //ritorna il vettore calcolato
      return result;
    }


    //----
    
    void add_scheme_to_list( std::string scheme ) {

      //Check if the scheme is valid (jump undefined stuff)
      if (scheme.find("undefined") == std::string::npos ) {        
        //Scheme Valid!!
        schemaList.push_back( scheme );
      }

    }
    

    void wait_new_hatp_plan( ) {

      //read plan from oprs message passer
      mpSocketRequest = external_register_to_the_mp_host_prot("seed_plan_listener", host_name.c_str(), 3300, STRINGS_PT);

      if ( mpSocketRequest < 0 ) 
        ROS_ERROR("Error connecting to mp");

      int length; 

      while( ros::ok() ) {

        char *sender = read_string_from_socket( mpSocketRequest, &length );
        char *message = read_string_from_socket( mpSocketRequest, &length );
        std::cout<<"OPRS message "<<message<<"\n";

        //decode the plan
        std::string msg_str( message );
        int start_index;        
        std::string plan = after_space( message, &start_index );
        hatp_2_list = hatp2vector(hatp2vector(plan)[2]);       

        // (. (. take (. PR2_ROBOT GLUE_BOTTLE .) .) 31121 (. .) .) -> pickup(glue)
        for( int i=0;  i < hatp_2_list.size(); i++ ) {  
          std::stringstream ss;
          std::string action_name = hatp2vector(hatp2vector( hatp_2_list[i])[0])[0];
       
          std::vector<std::string> param_list = hatp2vector(hatp2vector(hatp2vector( hatp_2_list[i])[0])[1]);
          if( param_list.size() == 0 ){
            ss << action_name;                    
          }
          else {

            ss << action_name << "(";

            for( int j=1; j < param_list.size(); j++){
            
              
              if( j < param_list.size() -1 ) {
              
                  if( action_name == "handover" ) {
                    if( LAAS2seed_param(param_list[j]) == "human" ) {
                      ss.str("");
                      action_name = "give";
                      ss << action_name << "(human,";
                    }
                    else if( LAAS2seed_param(param_list[j]) == "robot" ) {
                      ss.str("");
                      action_name = "receive";
                      ss << action_name << "(robot,";
                    }
                  }
                  else 
                    ss << LAAS2seed_param(param_list[j])<<",";
              }
              else {
                  
                  ss << LAAS2seed_param(param_list[j]);
      
              }
            }
            ss<<")";

          } 
          //std::cout << "ss: " << ss.str() << std::endl;
          add_scheme_to_list( ss.str() );          
        }


        std::cout << "HL_goal: " << HL_goal << std::endl;
        if( HL_goal != "" ) {
          std_msgs::String hl_str;
          hl_str.data = HL_goal;

          std::cout << "publih" << std::endl;
          goal_pub.publish( hl_str );
        }       
        
        new_plan = true;
        have_plan = true;
      }

    }

    void wait_and_generate_plan() {

      //sleep 3 sec and generate a new plan
      /*
      sleep( 1 );
      
      std::cout << "generate new plan!" << std::endl;
      
      schemaList.push_back( "take(bracket1)" );
      schemaList.push_back( "give(bracket1,human)" );
      schemaList.push_back( "receive(bracket1,human)" );      
      schemaList.push_back( "go(table)");
      schemaList.push_back( "place(bracket1,table)");      
      schemaList.push_back( "take(glue)");
      schemaList.push_back( "glue(glue,table)");
      schemaList.push_back( "place(glue,table)");
      schemaList.push_back( "point(table)");
      schemaLi-st.push_back( "clean(table)");
      */

      /*
      std::cout << "HL_goal: " << HL_goal << std::endl;
      if( HL_goal != "" ) {
        std_msgs::String hl_str;
        hl_str.data = HL_goal;

        std::cout << "publih" << std::endl;
        goal_pub.publish( hl_str );
      }
      */
      //schemaList.push_back( "take(bracket1)" );      
      //schemaList.push_back( "attachbracket(bracket1,table)");
      //new_plan = true;
      //have_plan = true;
      
    }

    PlanListenerBehaviour(std::string instance){
      setName(instance2vector(instance)[0]);
      setInstance(instance);
      setRtm(QUIESCENCE);
      new_plan = false;
      have_plan = false;
      //prendi l'indirizzo della mia istanza
      myAddress=WM->getNodesByInstance(instance)[0];

      //Init TODO: put in a standalone behaviour
      WMV.set< std::string >("pr2.hand","null");
      WMV.set< double >("pr2.handFree",1);      
      //WMV.set< double >("glue.onKit",1);
      //WMV.set< double >("bracket1.onKit",1);


      nh.param("/seed_2_0/hl_goal", HL_goal, std::string(""));
      std::cout << "goal : " << HL_goal << std::endl;
      goal_pub = nh.advertise< std_msgs::String > ("/goal", 0);

      if ( !SIMULATE_HATP ) 
        //start new plan wait thread
        boost::thread wait_new_hatp_plan_t( &PlanListenerBehaviour::wait_new_hatp_plan, this );            
      else 
        boost::thread wait_and_generate_plan_t( &PlanListenerBehaviour::wait_and_generate_plan, this );                  
    


    }

    bool perceptualSchema(){

      bool release=true;
    
      pthread_mutex_lock(&memMutex);


      if( have_plan ) {
     
        //---New plan incoming	
        if ( new_plan ) {

          for(int i=0; i<schemaList.size(); i++){
            std::cout<<schemaList[i]<<"\n";
          }      
          //step 0 - primo passo della sequenza
          step=0;
          //alloca il primo step
          myAddress->addSon(schemaList[step]);        
          //resetta il tempo d'attesa
          waitTime = INSTALL_WAIT_TIME;
          new_plan = false;  
          have_plan = false;    
        }
      }
      else {
        release = false;
      }

      replan=false;

      if(myAddress->amplification>0)
        myAddress->amplification--;

      this->setRtm(0.2);
      
      //se ho dei figli
      if(myAddress->son.size()!=0){

        //se il primo figlio ha raggiunto il goal
        if (myAddress->son[0]->goalStatus()) {
          std::cout << "hatpStream: " << myAddress->son[0]->instance << " done!\n";

          release = true;
        }
        //se è rilasciato  
        else if ( myAddress->son[0]->isWorking() ) {
          //continua col piano corrente
          release = false;
          //resetta il tempo d'attesa
          waitTime = INSTALL_WAIT_TIME;
        }
        //altrimenti (il primo figlio non è rilasciato e non ha raggiunto il goal)
        else {
          release=false;
          //lascia vero il releaser per il replan
        }
        
        if (release && waitTime > 0) {
          //allora decrementa il tempo d'attesa
          waitTime -= this->getRtm();
          //non attivare ancora lo schema motorio
          release = false;
        }
      }

      pthread_mutex_unlock(&memMutex);

      return release;
      //---
    }

    void motorSchema() {
    
      pthread_mutex_lock(&memMutex);
    
      //se ho un figlio
      if(myAddress->son.size()!=0){
    
        //rimuovilo
        remove(myAddress->son[0]);
      
        //se ho raggiunto un nuovo landmark
        if(replan){
          std::cout << this->getInstance() << " NEED REPLAN!\n";
        }
        //altrimenti (ho raggiunto il goal), se ho ancora step da svolgere
        else if(step!=schemaList.size()-1){
          
          //ancora azioni
          std::string strMessage="(AttentionalInterface.isDone FALSE)";
          char message[100];
          strcpy(message,strMessage.c_str());
          send_message_string(message,"OPRS_SUP");


          //alloca il prossimo step
          step++;
          WM_node* newStep = myAddress->addSon(schemaList[step]);
          newStep->amplification+=myAddress->amplification;
          //myAddress->son[0]->releaser.push_back("TRUE");
          //resetta il tempo d'attesa
          waitTime = INSTALL_WAIT_TIME;


        }
        else {
          std::cout << "ho finito!!" << std::endl;
          std::string strMessage="(AttentionalInterface.isDone TRUE)";
          char message[100];
          strcpy(message,strMessage.c_str());
          send_message_string(message,"OPRS_SUP");

        }




      }
      //altrimenti non ho figli
      else{
        remove(myAddress);

        //piano completo!
        std::cout << this->getInstance() << " DONE!\n";

        

      }

      pthread_mutex_unlock(&memMutex); 


            if(dead()) return;
   
    }

    
	private:

    std::vector<std::string> hatp_2_list, schemaList;
    WM_node* myAddress;
    int step;
    double waitTime;
    bool replan;
    std::vector <std::string> action_list;   
    bool new_plan, have_plan;
    int mpSocketRequest;

    ros::Publisher goal_pub;
    std::string HL_goal;
    ros::NodeHandle nh;
};


#endif
