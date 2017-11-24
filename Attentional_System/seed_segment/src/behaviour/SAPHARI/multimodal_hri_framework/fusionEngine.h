/* 
 * File:   fusionEngine.h
 * Author: hargalaten
 *
 * Created on 8 settembre 2014, 11.34
 */

#ifndef FUSIONENGINE_H
#define	FUSIONENGINE_H

#include "../../../seed_header.h"
#include "AlfredDialogueManager/POMDP.h"
#include "scfg/SCFG.h"
#include <bitset>
#include <algorithm>
#include "std_msgs/String.h"

//#define GESTURE_WAIT_TIME 10 //decimi di secondo
#define GESTURE_WAIT_TIME 5 //decimi di secondo
#define MODES 3 //numero di modalità
#define FUSION_WAIT_TIME 10 //decimi di secondo

//              NUOVA VERSIONE DEL FUSORE

/** dialogue(dial_name,source)
 * dial_name: è il nome del file xml da cui acquisire la politica
 * source: è il nome dello schema da cui acquisire la n-best delle osservazioni
 */
class FusionEngineBehaviour : public Behaviour {
public:
    FusionEngineBehaviour(std::string instance) {
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        obs.push_back(Observ("",1));
        gWaitTime=GESTURE_WAIT_TIME;
        fWaitTime=FUSION_WAIT_TIME;
        observed=false;
        
        //sender_obj = nh.advertise<std_msgs::String>("vrep/vrep_object", 1000);
        //sender_comm = nh.advertise<std_msgs::String>("vrep/vrep_command", 1000);

        sender_obj = nh.advertise<std_msgs::String>("multimodal_object", 1000);
        sender_comm = nh.advertise<std_msgs::String>("multimodal_command", 1000);

        g=new SCFG(SEED_HOME_PATH + "/src/behaviour/SAPHARI/multimodal_hri_framework/scfg/grammar");
        
        for(int i=0;i<MODES; i++){
            sentence.push_back( std::pair<std::string,double>("null",0) );
        }
        old_gesture="";
        
        WMV.set< double >(this->getInstance() + ".n",1);
    }

    bool perceptualSchema() {
        //bool rel=false;
        std::stringstream ss;
        
        pthread_mutex_lock(&memMutex);
        
        this->setRtm(0.1);
        
        if(dead()) return false;
        
        gestureList=sortNodes(WM->getNodesByName("gesture")); //sentence 0
        speechList=sortNodes(WM->getNodesByName("speech")); //sentence 1
        objectList=sortNodes(WM->getNodesByName("object")); //sentence 2
        
        //se il gesture è cambiato
        if(gestureList.size()!=0 && 
                old_gesture != gestureList[0]->instance && gestureList[0]->amplification>0.4){
//                && gestureList[0]->amplification != 0){
            //allora ho un nuovo gesto
            sentence[0].first=gestureList[0]->instance;
            sentence[0].second=WM->getInstanceMagnitude(gestureList[0]->instance);
            
            old_gesture=sentence[0].first;
            gWaitTime=GESTURE_WAIT_TIME;
            observed=false;
//            std::cout<<"fusionEngine: start gesture!\n";
            
        }
        //altrimenti se ho un gesto
        else if(sentence[0].first != "null" && !observed && gWaitTime>0){
            //aspetto di essere sicuro che non sia rumore
            gWaitTime--;
            
            fWaitTime++;
            
        }
        
        //aggiungo subito lo speech perché è discreto
        if(speechList.size()>0){
            sentence[1].first=speechList[0]->instance;
            sentence[1].second=speechList[0]->amplification;
//            std::cout<<"fusionEngine: have speech!\n";
        }
//        
//        if(objectList.size()>1){
//            std::cout<<"objs: "<<objectList[0]->instance<<" "<<objectList[1]->instance<<"\n";
//            std::cout<<"m: "<<WM->getInstanceMagnitude(objectList[0]->instance)<<" "<<WM->getInstanceMagnitude(objectList[1]->instance)<<"\n";
//        }
        
        //aggiungo subito l'ggetto
        if(objectList.size()>1 && 
                (WM->getInstanceMagnitude(objectList[0]->instance) > 
                WM->getInstanceMagnitude(objectList[1]->instance)) &&
                objectList[0]->instance!="object(null)"){
            sentence[2].first=objectList[0]->instance;
            sentence[2].second=WM->getInstanceMagnitude(objectList[0]->instance);
        }
        else{
            sentence[2].first="null";
            sentence[2].second=0;
        }
            
        //se almeno uno dei canali è pieno (gesto-parlato)
        if((sentence[0].first != "null" && gWaitTime<=0) || sentence[1].first != "null"){
            //invio la parola alla grammatica
//            std::stringstream ss;
            ss.str("");
            if(sentence[0].first!="null" && sentence[1].first=="null")
                ss<<sentence[0].first<<" null ";
            else  if(sentence[0].first!="null" && sentence[1].first!="null")
                ss<<sentence[0].first<<" ";
            
            if(sentence[1].first!="null" && sentence[0].first=="null")
                ss<<sentence[1].first<<" null ";
            else if(sentence[1].first!="null" && sentence[0].first!="null")
                ss<<sentence[1].first<<" ";
            
            ss<<sentence[2].first;
            
            
//            std::cout<<" o: "<<objectList[0]->instance<<" m: "<<WM->getInstanceMagnitude(objectList[0]->instance)<<"\n";
            
            std::cout<<"fusionEngine: "<<ss.str()<<"\n";
            
//            std::vector< std::vector< std::pair<std::string,double> > > combination = combineSentence(sentence);
//            for(int l=0; l<combination.size(); l++){
//                std::cout<<combination[l][0].first<<" "
//                        <<combination[l][1].first<<" "
//                        <<combination[l][2].first<<"\n";
//                        
//            }
            
//            obs=g->is_inGrammar(ss.str());
//            if(obs.size()>0){
//                rel=true;
//                observed=true;
//                
//                fWaitTime--;
//            }
//            else 
                fWaitTime--;
        }
        
        
//        if(sentence[0].first != "" && !observed){
//            waitTime--; //è passato un decimo di secondo
//            if(objectList.size()==1 || 
//                    (objectList.size() >1 && (WM->getInstanceMagnitude(objectList[0]->instance) >
//                                       WM->getInstanceMagnitude(objectList[1]->instance) ) ) ){
//                object=instance2vector(objectList[0]->instance)[1];
//                rel=true;
//            }
//            else if(objectList.size()!=0)
//                object="UNKNOWN";
//        }
            
        
        pthread_mutex_unlock(&memMutex);
        
        if(fWaitTime<=0)
            obs=g->is_inGrammar(ss.str());
        
        return (/*rel &&*/ fWaitTime<=0);

    }

    void motorSchema() {
        
        pthread_mutex_lock(&memMutex);

        this->setRtm(0.1);
        
        double pr2moving=WMV.get<double>("pr2.moving");
        
            if(obs.size() != 0 && !pr2moving){
                WMV.set< std::vector<Observ> >(this->getInstance() + ".nbest",obs);
                WMV.set< std::string >(this->getInstance() + ".obj",sentence[2].first);

                std::cout<<"ALFRED (fusionEngine) observed: "<<obs[0].GetCommand()<<" - "<< sentence[2].first <<"\n";
                //std::stringstream toSay;
                //toSay<<"espeak -v mb-en1 -s 130 \""<< "observed: "<<obs[0].GetCommand()<<" "<< sentence[2].first <<"\" 2> /dev/null";
                //system(toSay.str().c_str());
                observed=true;
                
                std::stringstream exe;
                std::string apps(obs[0].GetCommand());
                
                std::transform(apps.begin(), apps.end(), apps.begin(), ::tolower);
                
                
                if(sentence[2].first != "null"){
                    obj.data = instance2vector(sentence[2].first)[1];
                }
                else{
                    obj.data = "null";
                }
                comm.data = apps;
                
                for(int k=0; k<1; k++){
                sender_obj.publish(obj);
                sender_comm.publish(comm);
                }
                
                
                //inserimento delle intenzioni come nodi
                
                //WM_node *myAddress=WM->getNodesByInstance(this->getInstance())[0];
                //if(myAddress->son.size()!=0){
                //    remove(myAddress->son[0]);
                //}

                //cancellazione pezzotta
                WM_node *myAddress=WM->getNodesByInstance(this->getInstance())[0];
                if(myAddress->son.size()==1 && myAddress->son[0]->instance != "preparecoffee"){
                    remove(myAddress->son[0]);
                }
                else if(myAddress->son.size()>1){
                    remove(myAddress->son[1]);
                }
                
                std::cout<<"before "<<sentence[2].first<<"\n";
                
                if (apps=="stop") {
                    WM_node* newIntention = myAddress->addSon( intention2kuka(apps) );
                    newIntention->amplification=10;
                    
                } else {
                    if(sentence[2].first != "null" && sentence[2].first != "TEACH"  ){
                        apps = apps + "(" + instance2vector(sentence[2].first)[1] + ")";
                    }

                    std::cout<<"toKUKA: "<<apps<<"\n";
                    std::string kuka_behavior = intention2kuka(apps);
                    WM_node* kuka_node = myAddress->addSon( kuka_behavior );
                    std::cout<<"newSon: "<<kuka_behavior<<"\n";

                    if(kuka_behavior == "kuka(shutdown)" || kuka_behavior == "kuka(stop)" )
                        kuka_node->amplification += 10;
                }
//                myAddress->addSon(intention.str());
            }
            else{
                
                //std::stringstream toSay;
                //toSay<<"espeak -v mb-en1 -s 130 \" I dont understand \" 2> /dev/null";
                //system(toSay.str().c_str());
                
                std::cout<<"ALFRED (fusionEngine): NON HO CAPITO!\n";
            }
        
        obs.erase(obs.begin(),obs.end());

                //azzera tutti gli speech
                for(size_t i=0;i<speechList.size();i++)
                    remove(speechList[i]);
        
        //azzera tutti gli object
        for(size_t i=0;i<objectList.size();i++)
            if(objectList[i]->instance != "object(null)")
                remove(objectList[i]);
        //NB l'oggetto potrebbe anche non essere dimenticato, ma salvato in un
        //  old_obj che può essere provato nel successivo coclo di fusione
        
        for(size_t i=0;i<WM->getNodesByInstance("gestureRecognition")[0]->son.size(); i++){
            WM->getNodesByInstance("gestureRecognition")[0]->son[i]->amplification=0;
//            std::cout<<WM->getNodesByInstance("gestureRecognition")[0]->son[i]->instance<<" DUMPED!\n";
        }
            
//        printWM(WM);
        
        pthread_mutex_unlock(&memMutex);
        
        gWaitTime=GESTURE_WAIT_TIME;
        fWaitTime=FUSION_WAIT_TIME;
        
        old_gesture=sentence[0].first;
        
        sentence[0].first=sentence[1].first=sentence[2].first="null";
        
        sentence[0].second=sentence[1].second=sentence[2].second=0;
        
    }

    void start() {
        std::cout << "ALFRED: fusionEngine on! \n";
    }

    void exit() {
        std::cout << "ALFRED: fusionEngine off! \n";
    }
    std::string intention2kuka( std::string intention ){
        std::vector< std::string > intVect = instance2vector(intention);
        std::stringstream toKuka("");
        if(intVect.size() == 1 && intention == "open")
            toKuka << "kuka(gripper(open))";//"kukaGripper(open)";
        else if(intVect.size() == 1 && intention == "close")
            toKuka << "kuka(gripper(close))";//"kukaGripper(close)";
        else if(intVect.size() == 1 && intention == "stop")
            toKuka << "kuka(stop)";//"kukaStop";
        else if(intVect.size() == 1 && intention == "teach" )
            toKuka << "kuka(teach)";//"kukaTeach";
        else if(intVect.size() == 1 && intention == "done" )
            toKuka << "kuka(done)";//"kukaDone";
        else if(intVect.size() == 1 && intention == "shutdown" )
            toKuka << "kuka(shutdown)";//"kukaShutdown";
        else if(intVect.size() == 1 && intention == "repeat" )
            toKuka << "repeattask";//"kukaShutdown";
        //else if(intVect.size() > 1 && intVect[0] == "give")
        //    toKuka << "kukaGive("<<intVect[1]<<")";
        else if(intVect.size() > 1 && intVect[0] == "take")
            toKuka << "prepare"<<intVect[1];
        else if(intVect.size() > 1 && intVect[0] == "give")
            toKuka << "giveto("<<intVect[1]<<")";

        return toKuka.str();
    }
    std::string sentence2grammar(std::vector< std::pair<std::string,double> > sentence){
        std::stringstream ss;
        if(sentence[0].first!="null" && sentence[1].first=="null")
            ss<<sentence[0].first<<" null ";
        else  if(sentence[0].first!="null" && sentence[1].first!="null")
            ss<<sentence[0].first<<" ";
            
        if(sentence[1].first!="null" && sentence[0].first=="null")
            ss<<sentence[1].first<<" null ";
        else if(sentence[1].first!="null" && sentence[0].first!="null")
            ss<<sentence[1].first<<" ";
            
        ss<<sentence[2].first;
        return ss.str();
    }
    
    std::vector< std::vector< std::pair<std::string,double> > > combineSentence(std::vector< std::pair<std::string,double> > in){
        std::vector< std::vector< std::pair<std::string,double> > > result;
        
        //vettore degli indici degli elementi diversi da NULL
        std::vector< int > k;
        
        //calcolo il numero degli elementi diversi da NULL ( con k.size() sempre minore/uguale di in.size() )
        for(size_t i=0; i<in.size(); i++){
            //se l'i-esimo elemento è NULL contalo
            if(in[i].first != "null")
                k.push_back(i);
        }
        
        //inizializzo la matrice delle combinazioni a NULL
        
        //per ogni combinazione
        for(size_t i=0; i< pow(2,k.size()); i++){
            
            //creo una sentence d'appoggio
            std::vector< std::pair<std::string,double> > app;
            
            //la inizializzo a NULL
            for(size_t j=0; j<in.size(); j++)
                app.push_back( std::pair<std::string,double>("null",0) );
            
            //genero la combinazione
            std::string s = std::bitset< 64 >( i ).to_string();
            
            //per ogni elemento diverso da NULL nel vettore di partenza
            for(size_t j=0; j<k.size(); j++){
                //se è l'elemento scelto lo inserisco
                if( s[64-(j+1)] == '1' ){
                    app[ k[j] ].first = in[ k[j] ].first;
                    app[ k[j] ].second = in[ k[j] ].second;
                }
            }
            
            //aggiungo la combinazione al result
            result.push_back(app);
        }
        
        return result;
        
    }
protected:
    std::vector<WM_node *> gestureList;
    std::vector<WM_node *> objectList;
    std::vector<WM_node *> speechList;
    std::vector<Observ> obs;
    std::string old_gesture;
    double gWaitTime,fWaitTime;
    bool observed;
    std::vector< std::pair<std::string,double> > sentence;
    SCFG *g;
    ros::NodeHandle nh;
    ros::Publisher sender_obj, sender_comm;
    std_msgs::String obj,comm;
};


#endif	/* FUSIONENGINE_H */
