#ifndef _SAPHARI_B_CLASS_
#define _SAPHARI_B_CLASS_

#include "../../seed_header.h"
#include "tf/transform_broadcaster.h"
#include "TooN/TooN.h"

#define SIMULATE_HATP   0   // 1: simulate an immediate output from HATP (directly to the schemalist)

#define INSTALL_WAIT_TIME 0
#define MAX_LOCATION_DISTANCE 640
#define ACTION_WAIT 2

#define GAIN_DIST_MIN 0.2
#define GAIN_DIST_MAX 2.0

using namespace TooN;

std::string return_ok="(AttentionalInterface.report OK)";
std::string host_name="maxc2";
  //input pos, R matrix
  inline void tf_publish( TooN::Vector<3> p, TooN::Matrix<3> R, std::string ref_frame, std::string target_frame) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3( p[0], p[1], p[2]) );
    tf::Quaternion q(0, 0, 0, 1);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), ref_frame, target_frame));
  }

std::string LAAS2seed_param( std::string LAAS_param ) {
  std::string SEED_param;

  if( LAAS_param == "BRACKET_1" ) {
    SEED_param = "bracket1";
  }
  else if( LAAS_param == "BRACKET_2" ) {
    SEED_param = "bracket2";
  }
  else if( LAAS_param == "BRACKET_3" ) {
    SEED_param = "bracket3";
  }
  else if( LAAS_param == "GLUE_BOTTLE" ) {  
    SEED_param = "glue";
  }
  else if( LAAS_param == "WORK_LOCATION_1" ) {  
    SEED_param = "location1";
  }
  else if( LAAS_param == "WORK_LOCATION_2" ) {  
    SEED_param = "location2";
  }
  else if( LAAS_param == "WORK_LOCATION_3" ) {  
    SEED_param = "location3";
  }
  else if( LAAS_param == "STOCK_TABLE" ) {  
    SEED_param = "table";
  }
  else if( LAAS_param == "ASSEMBLY_SURFACE_1" ) {  
    SEED_param = "surface";
  }
  else if( LAAS_param == "ASSEMBLY_SURFACE_2" ) {  
    SEED_param = "surface2";
  }
  else if( LAAS_param == "ASSEMBLY_SURFACE_3" ) {  
    SEED_param = "surface3";
  }
  else if( LAAS_param == "PLACEMENT_WL1_1" ) {  
    SEED_param = "placementwl11";
  }
  else if( LAAS_param == "PLACEMENT_WL1_2" ) { 
    SEED_param = "placementwl12";
  }
  else if( LAAS_param == "PLACEMENT_WL2_1" ) {  
    SEED_param = "placementwl21";
  }
  else if( LAAS_param == "PLACEMENT_WL2_2" ) { 
    SEED_param = "placementwl22";
  }
  else if( LAAS_param == "PLACEMENT_WL3_1" ) { 
    SEED_param = "placementwl31";
  }
  else if( LAAS_param == "PLACEMENT_WL3_2" ) {  
    SEED_param = "placementwl32";
  }
  else if( LAAS_param == "PLACEMENT_ST_1" ) {  
    SEED_param = "placementst1";
  }
  else if( LAAS_param == "PLACEMENT_ST_2" ) {  
    SEED_param = "placementst2";
  }
  else if( LAAS_param == "PLACEMENT_ST_3" ) { 
    SEED_param = "placementst3";
  }
  else if( LAAS_param == "PLACEMENT_ST_4" ) { 
    SEED_param = "placementst4";
  }
  else if( LAAS_param == "PR2_ROBOT" ) {
    SEED_param = "robot";
  }

  else if( LAAS_param == "HERAKLES_HUMAN1" ) {
    SEED_param = "human";
  }

  return SEED_param;
}

//
std::string SEED2laas_param( std::string SEED_param ) {
  std::string LAAS_param;

  if( SEED_param == "bracket1" ) {
    LAAS_param = "BRACKET_1";
  }
  else if( SEED_param == "bracket2" ) {
    LAAS_param = "BRACKET_2";
  }
  else if( SEED_param == "bracket3" ) {
    LAAS_param = "BRACKET_3";
  }
  else if( SEED_param == "glue" ) {  
    LAAS_param = "GLUE_BOTTLE";
  }
  else if( SEED_param == "location1" ) {\  
    LAAS_param = "WORK_LOCATION_1";
  }
  else if( SEED_param == "location2" ) {  
    LAAS_param = "WORK_LOCATION_2";
  }
  else if( SEED_param == "location3" ) {  
    LAAS_param = "WORK_LOCATION_3";
  }
  else if( SEED_param == "table" ) {  
    LAAS_param = "STOCK_TABLE";
  }
  else if( SEED_param == "surface" ) {  
    LAAS_param = "ASSEMBLY_SURFACE_1";
  }
  else if( SEED_param == "surface2" ) {  
    LAAS_param = "ASSEMBLY_SURFACE_2";
  }
  else if( SEED_param == "surface3" ) {  
    LAAS_param = "ASSEMBLY_SURFACE_3";
  }
  else if( SEED_param == "placementwl11" ) {  
    LAAS_param = "PLACEMENT_WL1_1";
  }
  else if( SEED_param == "placementwl12" ) { 
    LAAS_param = "PLACEMENT_WL1_2";
  }
  else if( SEED_param == "placementwl21" ) {  
    LAAS_param = "PLACEMENT_WL2_1";
  }
  else if( SEED_param == "placementwl22" ) { 
    LAAS_param = "PLACEMENT_WL2_2";
  }
  else if( SEED_param == "placementwl31" ) { 
    LAAS_param = "PLACEMENT_WL3_1";
  }
  else if( SEED_param == "placementwl32" ) {  
    LAAS_param = "PLACEMENT_WL3_2";
  }
  else if( SEED_param == "placementst1" ) {  
    LAAS_param = "PLACEMENT_ST_1";
  }
  else if( SEED_param == "placementst2" ) {  
    LAAS_param = "PLACEMENT_ST_2";
  }
  else if( SEED_param == "placementst3" ) { 
    LAAS_param = "PLACEMENT_ST_3";
  }
  else if( SEED_param == "placementst4" ) { 
    LAAS_param = "PLACEMENT_ST_4";
  }

  return LAAS_param;
}

/* 
 *  pr2Behaviou class
 *  
 *  Metodi implementati:
 *      - motorScema(): invia le informazioni ad OPRS (tramite bridge) e
 *          gestisce autonomamente il conflitto con gli altri processi
 *          controllando l'accesso alla variabile "pr2.action".
 *      - start(): vuoto.
 *      - exit(): disconnette il behavior da OPRS.
 * 
 *  Metodi virtuali:
 *      - perceptualSchema(): deve stabilire i parametri ed invocare
 *          il metodo updateRtm per calcolare l'enfasi, il valore di ritorno
 *          booleano viene messo in and con il resto del releaser fornito dalla
 *          struttura della WM.
 *      - update_wmv(): deve modificare i valori delle variabili in base
 *          all'azione eseguita da PR2 (eg. azione "give" comporta che la 
 *          variabile "pr2.handFree" diventi vera).
 *      - oprs_message(): restituisce la std::string che viene inviata ad OPRS
 *          per avviare il comando. Usare seed2oprs() per convertire le
 *          stringhe (position e target) alla sintassi di OPRS.
 *  
 */
class pr2Behaviour : public Behaviour{
  public:   


    void ok_cb( sensor_msgs::Joy joy_msgs ) {
      if( joy_msgs.buttons[3] == 1)
        success = true;
    }

    pr2Behaviour() {
        //tempo di attesa di default
        toWait = ACTION_WAIT;
        //nome del kernel di default
        oprsKernel = "OPRS_SUP";


        if ( SIMULATE_HATP ) {
          success = false;
          ok_sub = nh.subscribe( "/joy", 0, &pr2Behaviour::ok_cb, this);
        }
    }
    void motorSchema(){


        //setta messaggio di ritorno a not-a-return di default
        oprsReturn="NaR";
        
        pthread_mutex_lock(&memMutex);
        bool ack=false;
        //prova a conquistare la variabile

        imWinning=send("plan_listener","pr2.action",this->getInstance());
        //std::cout<<this->getInstance()<<"... "<<imWinning<<"\n";
        //se sto vincendo la variabile ed il tempo di attesa non è finito
        
        if(imWinning==1 && toWait>=0){

            //aspetta (decrementa il tempo da attendere)
            toWait--;
            //WMV.set<double>("pr2.action.count",0);
        }

        //altrimenti, se sto vincendo la variabile (e il tempo di attesa è finito)
        else if(imWinning==1){

            
            //std::cout<<"PR2: "<<this->getInstance()<<" to OPRS\n";
            //send to oprs
            std::stringstream ss;
            //ss<<"( AttentionalInterface.execute"<< action <<" "<<target<<" "<<position<< " "<<mpSender<< " )";
            char app[100];
            strcpy(app,oprs_message().c_str());
            
            if( !SIMULATE_HATP )
              send_message_string(app, oprsKernel.c_str());
            
            //il pr2 è in movimento
            WMV.set<double>("pr2.moving",1);
            //messaggio ricevuto
            ack=true;
            
        }
        //altrimenti (non sto vincendo)
        else
            //resetta il tempo di attesa
            toWait=ACTION_WAIT;
        
        //WMV.set< std::string >("bwArm.action",this->getInstance());
        pthread_mutex_unlock(&memMutex);
        
        //se sto vincendo, ho aspettato ed ho inviato il messaggio
        if (imWinning==1 && toWait<0 && ack) {
          //aspetta la risposta di OPRS (le read sono bloccanti)
          int intOprs;
      
          if( !SIMULATE_HATP ) { 
            //scarta la prima risposta (dovrebbe essere un ack)
            oprsReturn = read_string_from_socket(mpSocket,&intOprs);
            //aspetta la risposta di OPRS (esito dell'azione inviata)
            oprsReturn = read_string_from_socket(mpSocket,&intOprs);              
          }
          else {
          
            //wait success from joypad
            if( success ) {
              oprsReturn = return_ok;
              success = false;
            }
          }            
        } 
        
        pthread_mutex_lock(&memMutex);
        //se OPRS ritorna "ok"
        if( oprsReturn == return_ok ){
            //la azone è stata eseguita con successo
            
            //libera la variabile
            WMV.set<double>("pr2.action.count",0);
            WMV.set< std::string >("pr2.action","null");
            WM->getNodesByInstance("plan_listener")[0]->winner="";
            
            //aggiorna lo stato delle variabili in relazione al comando eseguito
            update_wmv();
            
            pthread_mutex_unlock(&memMutex);
            std::cout<<"PR2: "<<this->getInstance()<<" done!\n";
      
        }
        //altrimenti, se OPRS ritorna qualcosa diversa da NaR
        else if(oprsReturn!="NaR")
            //segnala un ritorno inatteso
            std::cout<<this->getInstance()<<" unespexted OPRS return: "<<oprsReturn<<"\n";
        
        //WMV.set< std::string >("bwArm.action",this->getInstance());
        pthread_mutex_unlock(&memMutex);


        
    }
    virtual void update_wmv()=0;
    virtual std::string oprs_message()=0;
    void start(){
    }
    void exit(){
        close(mpSocket);
    }
protected:
    //target dell'azione da inviare ad OPRS
    std::string target;
    //posizione nel quale effettuare l'azione
    std::string position;

    //agente coinvolto
    std::string agent;

    
    //true se sto vincendo la variabile condivisa "pr2.action"
    bool imWinning;
    //messaggio di ritorno da OPRS (risultato dell'azione)
    std::string oprsReturn;
    //countdown di attesa per essere sicuri che non ci siano cambiamenti nel
    //vincitore di una variabile
    double toWait;
    
    //socket del bridge OPRS
    int mpSocket;
    //nome del kernel OPRS
    std::string oprsKernel;
    //nome identificativo del behavior che comunica con OPRS
    std::string mpSender;

    ros::NodeHandle nh;
    ros::Subscriber ok_sub;
    bool success;
};

#endif
