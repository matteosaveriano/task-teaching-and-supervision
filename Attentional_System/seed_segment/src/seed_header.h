/* 
 * File:   seed_header.h
 * Author: hargalaten
 *
 * Created on 12 dicembre 2012, 15.45
 */

#ifndef SEED_HEADER_H
#define	SEED_HEADER_H

#include <cstdlib>
#include <iostream>
#include <pthread.h>
#include <fstream>
#include <string>
#include <map>
#include <math.h>
#include <time.h>
#include <boost/lexical_cast.hpp>
#include "boost/random.hpp"

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt32.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "opencv/cv.h"
#include "opencv2/opencv.hpp"
#include "opencv2/ml/ml.hpp"
#include "eclipseclass.h"
#include <signal.h>
#include <execinfo.h>

    
#define QUIESCENCE 2
#define DEFAULT 1
#define MSECOND 1000000
#define PRECISION 100
#define MAXFREQ 0.1
#define MAXMAG 10
#define DEFAULT_REFRACTORY_TIME 3



extern pthread_mutex_t memMutex;

extern std::string SYS_HOME_PATH;
extern std::string SEED_HOME_PATH;

// Utility functions
std::string functor2string(EC_word);
std::vector<std::string> instance2vector(std::string);    
void printList(EC_word);
std::vector<std::string> readList(EC_word);
EC_word writeList(std::vector<std::string>);
float r2d( float rad );
float d2r( float deg );

      
// Class definitions   
// Working Memory (WM) variables' hash map
class WM_varMap {
public:

//    void set(std::string id, double value){
//        d[id]=value;
//    }
//    void set(std::string id, int value){
//        d[id]=value;
//    }
//    void set(std::string id, std::string value){
//        s[id]=value;
//    }
//    void set(std::string id, bool value){
//        b[id]=value;
//    }

  template<class T>
  void set(std::string id, T value)
  {
    //std::cout<<"setting "<<id<<": "<< value <<"\n";
    if(p.find(id)!=p.end())
      //free(p.find(id)->second);
      delete p.find(id)->second;

    T *point=new T();
    *point=value;
    p[id]= (void*) point;      
  }   
//    void set(std::string id, void* value){
//        p[id]=value;
//    }
//    void get(std::string id, double *value){
//        *value=d[id];
//    }
//    void get(std::string id, int *value){
//        *value=d[id];
//    }
//    void get(std::string id, std::string *value){
//        *value=s[id];
//    }
//    void get(std::string id, bool *value){
//        *value=b[id];
//    }
    template<class T>
    T get(std::string id){
        //std::cout<<"getting "<<id<<"\n";
        if(p.find(id)!=p.end() && p[id]!=NULL){
            //std::cout<<"getDone "<<id<<": "<<*((T *) p[id])<<"\n";
            return *((T *) p[id]);
        }
        else{
            //std::cout<<"getDone (default) "<<id<<": "<<T()<<"\n";
            return T();
        }
    }
private:   
    /* Unique map for all WM variables */
    std::map<std::string,void*> p;
};

extern WM_varMap WMV;

// WM node (of the tree) class
class WM_node{
public:
    WM_node(std::string newInstance, WM_node* instanceFather){
        name=instance2vector(newInstance)[0];
        instance=newInstance;
        //releaser.push_back("TRUE");
        father=instanceFather;
        abstract=true;
        expanded=false;
        ltMagnitude=magnitude=rtm=DEFAULT;
        amplification=0;
        goalCount=0;
        teleological=false;
        amplified=false;
        winner="";
        oldWinner="";
        winnerRtm=QUIESCENCE;
        winnerMag=0;
        freezed=false;
        internalReleaser=true;
    }
    // Name of the schema
    std::string name;
    // Instance of the schema (ie. name+parameters)
    std::string instance;
    // Releaser: list of WM variables (in AND)
    std::vector<std::string> releaser;
    // Magnitude (computed by the WM)
    double magnitude;
    // Magnitude of the schema (acquired by the semantic function)
    double ltMagnitude;
    // Amplification of the magnitude (acquired during the execution)
    double amplification;
    // Number of times a goal is achieved
    double goalCount;
    // Current rithm of the node (behaviour)
    double rtm;
    // List of child nodes
    std::vector< WM_node*> son;
    // Pointer to the father node
    WM_node *father;
    // Determine if the schema is abstract or not
    bool abstract;
    // TRUE if the node has been expanded
    bool expanded;
    // TRUE if the node is goal oriented
    bool teleological;
    // TRUE if the node has to be amplified
    bool amplified;
    // goal: list of WM variables (in AND)
    std::vector<std::string> goal;
    // Winning node (winner-take-all strategy used to choose among the active nodes)
    std::string winner;
    // Old winning node
    std::string oldWinner;
    // Period of the winning node
    double winnerRtm;
    // Magnitude of the winning node
    double winnerMag;
    // List (FIFO) of competeng nodes. The head of the list contains the winner
    std::vector<std::string> contenders;
    // Fade value. The node is canceled if fading = 0
    double fading;
    // TRUE if the node is frozen (ie. the node cannot achivate the motor schema)
    bool freezed;
    // TRUE if the releaser of the behaviour is true
    bool internalReleaser;
    /** Function that transforms a sub-tree into a list of nodes
     * 
     * @return list of nodes in the sub-tree
     */
    std::vector<WM_node *> tree2list(){
        std::vector<WM_node *> nodeList;
        nodeList.push_back(this);
        for(int i=0; i<this->son.size(); i++){
            std::vector<WM_node *> subList=this->son[i]->tree2list();
            nodeList.insert( nodeList.end(), subList.begin(), subList.end() );
        }
        
        return nodeList;
    }
    
    /** Function to calculate the magnitude of the node
     * 
     * @param instanceToFind instance of the considered behaviour 

     * @return total magnitude of all the active behaviours (released)
     *  having "instanceToFind" as instance
     */
    double getInstanceMagnitude(std::string instanceToFind){
        double totalMagnitude=0;
        // Do not sum up the instances 
        return 1;
        // If released then ...
        if(this->releaserStatus() && !this->goalStatus()){
            // If this->instance is the desired instance
            if(this->instance == instanceToFind)
                // Store the magnitude of the desired instance
                totalMagnitude+=magnitude+amplification;   
            // Check if the children have instances of the same behaviour 
            for(int i=0;i<son.size();i++)
                // Sum the magnitude of all the instances in the sub-tree
                totalMagnitude+=son[i]->getInstanceMagnitude(instanceToFind);
        }
        // Return the total magnitude
        return totalMagnitude;
    }
    /** Function to check if the goal is achieved
     * 
     * @return TRUE if the goal of the node has been achieved
     */
    bool goalStatus(){
        int i=0;
        double var;
        char c;
        bool isTrue=true;
        std::string app;
        // If the node is not teleological the goal is always FALSE
        if(!this->teleological) return false;
        
	while(i<goal.size() && isTrue){
            var = WMV.get<double>(goal[i]);
            // If the i-th element is negated
            if(goal[i][0]!='-' && 
                    // and it is FALSE in the WM then the goal is FALSE
                    var==0) isTrue=false;
            //altrimenti se è negato
            else if(goal[i][0]=='-'){
                //apri uno stream
                std::stringstream ss(goal[i]);
                //scarta il not (ie. il simbolo -)
                ss>>c>>app;
                var = WMV.get<double>(app);
                //se nella memoria è vero allora il goal è falso
                if(var==1) isTrue=false;
            }
            i++;
        }
        //ritorna lo stato del goal
        return isTrue;
    };
    /** funzione di controllo del releaser per il singolo nodo
     * 
     * @return TRUE se il releaser del nodo è vero
     */
    bool releaserStatus(){
        int i=0;
        double var;
        char c;
        bool isTrue=true;
        std::string app;
        //mentre il vettore non è finito ed il releaser è vero
        while(i<releaser.size() && isTrue){
            var = WMV.get<double>(releaser[i]);
            //se l'iesimo elemento non è negato
            if(releaser[i][0]!='-' && 
                    //ed e falso nella memoria allora il releaser è falso
                    var==0) isTrue=false;
            //altrimenti se è negato
            else if(releaser[i][0]=='-'){
                //apri uno stream
                std::stringstream ss(releaser[i]);
                //scarta il not (ie. il simbolo -)
                ss>>c>>app;
                var = WMV.get<double>(app);
                //se nella memoria è vero allora il releaser è falso
                if(var==1) isTrue=false;
            }
            i++;
        }
        //ritorna lo stato del releaser
        return isTrue;
    };
    /** funzione per il controllo dei releaser sul ramo
     * 
     * @return TRUE se il ramo del nodo ha releaser veri
     * 
     * si combina con isreleased() per calcolare il valore dei
     * releaser per ogni nodo del ramo di cui quello attuale è
     * foglia
     */
    bool isBranchReleased(){      
        //se il mio releaser è vero ed ho un padre ritorna il suo releaser
        if(this->releaserStatus() && !this->goalStatus() && father!=NULL) return father->isBranchReleased();
        //altrimenti se sono rilasciato ma non ho un padre ritorna TRUE
        else if(this->releaserStatus() && !this->goalStatus() && father==NULL) return true;
        //altrimenti non sono rilasciato
        return false;
    };
    /**
     * Funzione di controllo del releaser su tutta la memoria
     * 
     * @param toFind istanza da cercare
     * @return true se esiste almeno una istanza con releaser vero, 
     *          false altrimenti
     */
    bool isReleased(std::string toFind){
        //se io sono il nodo cercato ritorno me stesso
        if(this->instance == toFind && this->isBranchReleased()) return true;         
        //altrimenti chiedi ai figli il nodo da cercare
        else for(int i=0;i<son.size();i++)
                if(son[i]->isReleased(toFind))
                    return true;
        //altrimenti il nodo non c'è
        return false;
    }
    /**
     * attenua il sottoalbero, sottraendo all'importanza dei nodi la propria
     * magnitudine
     */
    void mitigate(){
        this->amplification-=this->magnitude;
        if(this->amplification<0) this->amplification=0;
        for(int i=0;i<(this->son).size();i++)
            (this->son[i])->mitigate();
    }
    /**
     * amplifica il sottoalbero, maggiorando l'importanza dei nodi con la 
     * propria magnitudine
     * 
     */
    void amplify(double factor){
        this->amplification+=factor;
        for(int i=0;i<(this->son).size();i++)
            (this->son[i])->amplify(factor);
    }
    /**
     * cerca ed amplifica tutti i nodi che hanno appena raggiunto il goal
     */
    void amplifyNodes(){
        
        //controlla i figli
        for(int i=0;i<son.size();i++){
            son[i]->amplifyNodes();
        }
        
        //se il nodo è teleologico
        if(this->teleological){
            //se il goal è vero ma il nodo non è stato amolificato
            if(this->goalStatus() && !this->amplified){
                //std::cout<<this->instance<<" success!\n";
                //se è la prima volta che raggiunge il goal
                if(this->goalCount==0){
                    //consideralo amplificato
                    this->amplified = true;
                    this->goalCount++;
                    //amplifica il sottoalbero in esso radicato
                    (this->father)->amplify(this->magnitude);
                }
                //altrimenti (ie. è stato amplificato altre volte)
                else{
                    //consideralo amplificato
                    this->amplified = true;
                    this->goalCount++;
                }
            }
            //altrimenti se il goal è falso ma il nodo è amplificato
            else if(!this->goalStatus() && this->amplified)
                //segnalalo come non amplificato
                this->amplified=false;
        }
                
    }
    /**
     * Scorre l'intera WM e controlla lo stato del fading.
     * Ogni nodo il cui fading è nullo viene rimosso dalla WM
     */
    void forgetNodes(){
        if(this->fading <= 0){
            //remove(this);
        }
        else {
            for (int i = 0; i < son.size(); i++) {
                son[i]->forgetNodes();
            }
        }
    }
    //ritorna il primo nodo espandibile del sottoalbero
    WM_node* getExpandableNode(){
        int i=0;
        bool rel;
        //se il mio releaser è falso torna NULL
        WM_node *expNode=NULL;
        
        rel=this->releaserStatus();
        //e non sono espanso torna me stesso 
        /*espandimi anche se non sono vero... ora espandi anche i figli [ma non espandere i miei figli]*/
        if(!expanded /*&& rel*/) expNode=this;
        //altrimenti se sono stato espanso controlla i figli
        else if(expanded /*&& rel*/)
            while(i<son.size() && (expNode = son[i]->getExpandableNode()) == NULL) i++;
        
        return expNode;
    };
    //allocazione ed aggiunta di un nuovo figlio
    WM_node* addSon(std::string sonInstance){
        WM_node *newSon=new WM_node(sonInstance,this);
        son.push_back(newSon);
        return newSon;
    };
    //cerca e restituisci un nodo qualsiasi, che istanzia "name"
    std::vector<WM_node *> getNodesByName(std::string name)
    {
        std::vector< WM_node*> result,sonResult;
        //se io sono il nodo cercato ritorno me stesso
        if(this->name == name) result.push_back(this);         
        //altrimenti chiedi ai figli il nodo da cercare
        for(int i=0;i<son.size();i++){
            sonResult=son[i]->getNodesByName(name);
            if(sonResult.size()!=0)
                for(int j=0;j<sonResult.size();j++)
                    result.push_back(sonResult[j]);
        }
        //altrimenti il nodo non c'è
        return result;
    };
    /** cerca e restituisce tutti i nodi identificati dall'istanza
     * 
     * @param name istanza da cercare nella memoria
     * @return lista di tutti i nodi aventi istanza "name"
     */
    std::vector< WM_node*> getNodesByInstance(std::string name)
    {
        std::vector< WM_node*> result,sonResult;
        //se io sono il nodo cercato ritorno me stesso
        if(this->instance == name) result.push_back(this);         
        //altrimenti chiedi ai figli il nodo da cercare
        for(int i=0;i<son.size();i++){
            sonResult=son[i]->getNodesByInstance(name);
            if(sonResult.size()!=0)
                for(int j=0;j<sonResult.size();j++)
                    result.push_back(sonResult[j]);
        }
        //altrimenti il nodo non c'è
        return result;
    };
    /** funzione di controllo della presenza di un behaviour (concreto) sveglio
     * 
     * @param name istanza da ricercare
     * @return TRUE se esiste un behaviour "name" già istanziato (sveglio)
     */
    bool isAwake(std::string name)
    {
        //se io sono il nodo cercato e sono sveglio ritorna true
        if(this->instance == name && this->expanded) return true;   
        //controlla se tra i miei figli vi sono istanze sveglie
        for(int i=0;i<son.size();i++)
            if(son[i]->isAwake(name)) return true;
        //altrimenti torna false
        return false;
    };

    void updateMagnitude() {
        WM_node *f = this->father;
        double sons = 0;
        if (f != NULL) {
            this->magnitude = ((f->magnitude)*(this->ltMagnitude));
        }
        for (int j = 0; j<this->son.size(); j++)
            this->son[j]->updateMagnitude();
    }
    void tic(){
        this->fading=this->magnitude + this->amplification;
    }
    void ticBranch(){
        this->tic();
        if(this->father != NULL)
            this->father->ticBranch();
    }
    /**
     * controlla se il sottoalbero ha almeno un nodo che esegue
     * delle operazioni
     * 
     * @return true se esiste nel sottoalbero almeno un nodo
     *  concreto con releaser vero
     */
    bool isWorking() {
        
        //se ho raggiunto il goal
        if(this->goalStatus()){
            //allora non sto lavorando
            return false;
        }
        //se il mio releaser è falso e sono stato espanso
        else if(!this->releaserStatus()){
            //allora non sto lavorando
            return false;
        }
        //se non sono stato ancora espanso
        else if(!this->expanded){
//            std::cout<<this->instance<<"->isWorking: !expanded\n";
            //allora sto lavorando
            return true;
        }
        //altrimenti, se sono astratto (ed ho releaser vero)
        else if(this->abstract){
            //controllo se tra i miei figli c'è un nodo concreto rilasciato
            for (int i = 0; i < this->son.size(); i++){
                //se trovo un nodo concreto rilasciato
                if(this->son[i]->isWorking()){
//                    std::cout<<this->instance<<"->isWorking: sonWork\n";
                    //allora sto lavorando
                    return true;
                }
            }
            //se non trovo nodi concreti rilasciati allora non sto lavorando
            return false;
        }
        //se non sono in nessuno dei casi precedenti
        else{
//            std::cout<<this->instance<<"->isWorking: concrete&released\n";
            //allora sono concreto ed attivo, quindi sto lavorando
            return true;
        }
    }
};

extern WM_node *WM;



//FUNZIONI GLOBALI
    
    void printSchemaList(EC_word);;
    
    bool wakeUp(std::string, std::string);

    void loadSemantics(EC_word ,WM_node*);
    
    void printWM (WM_node *);
    
    void remove(WM_node * node);
    
    void engineSend(double,double,double,std::string);
    
    void *execution(void *);
    
    bool dead();
    
    std::vector<WM_node *> sortNodes(std::vector<WM_node *>);

class Behaviour{
public:
    Behaviour(){
        oldPeriod=0;
        oldStimulus=0;
    }
    //funzione di aggiornamento del ritmo
    //virtual double updateRtm(double)=0;
    //funzione di uscita del behaviour
    virtual void exit()=0;
    //funzione di ingresso del behaviour
    virtual void start()=0;
    //schema percettivo del behaviour (ritorna lo stato del releaser interno)
    virtual bool perceptualSchema()=0;
    //schema motorio del behaviour
    virtual void motorSchema()=0;
    //setta il ritmo del behaviour
//    void updateRtm(double affordance,double max,double min){
//        if(affordance<max && affordance-min>0.1)
//            setRtm((affordance-min)/(getMagnitude()*(max-min)));
//        else if(affordance>=max)
//            setRtm(DEFAULT);
//        else 
//            setRtm(MAXFREQ/getMagnitude());
//    }
//    void updateRtm(double affordance,double max,double min){
//        if(getMagnitude()<10)
//            if(affordance-min>1)
//                if((affordance-min)/(getMagnitude()*(max-min))>1)
//                    setRtm(DEFAULT);
//                else
//                    setRtm((affordance-min)/(getMagnitude()*(max-min)));
//            else
//                setRtm((1/(max-min)*getMagnitude()));
//        else
//            if(affordance-min>1)
//                if((affordance-min)/(10*(max-min))>1)
//                    setRtm(DEFAULT);
//                else
//                    setRtm((affordance-min)/(max-min)*10);
//            else
//                setRtm(0.01);
//    }
    void updateRtm(double affordance,double max,double min){
        //retta passante per (min,0.1) e (max,1)
        double m=9/((max-min)*10);
        double q=-((9*min)/(10*(max-min)))+0.1;
        double mag=getMagnitude();
        if(mag<10)
            if(affordance>min)
                if(((m*affordance)+q)/mag>1)
                    setRtm(DEFAULT);
                else
                    setRtm(((m*affordance)+q)/mag);
            else
                setRtm(((m*min)+q)/mag);
        else
            if(affordance>min)
                if(((m*affordance)+q)/10>1)
                    setRtm(DEFAULT);
                else
                    setRtm(((m*affordance)+q)/10);
            else
                setRtm(0.01);
        
        this->updated=true;
    }
    //funzione di aggiornamento del periodo con legge di weber e soglia
    // il max diventa la soglia oltre il quale lo stimolo viene percepito
    //ATTENZIONE: qui il ritmo non viene integrato con la magnitudine
    void updateRtm_weber2(double newStimulus,double maxStimulus,double minStimulus){
        //retta passante per (min,0.1) e (max,1)
        //double newPeriod=0.01;
        double newPeriod=1;
        double newRelevance;
        
        //k neutra per prova
        k_weber=1;
        
        if(oldStimulus!=0 && newStimulus<maxStimulus) {
            
            if(newStimulus!=0 && newStimulus <= oldStimulus) //&& newStimulus > minStimulus)
                newRelevance = k_weber * ( (oldStimulus - newStimulus) / newStimulus);
            else if(newStimulus > oldStimulus)
                newRelevance = - k_weber * ( (newStimulus - oldStimulus) / oldStimulus);
            else
                newRelevance = oldPeriod;
            
            newPeriod = oldPeriod - newRelevance;

            if (newPeriod < 0.01)
                newPeriod=0.01;
            else if (newPeriod > 1)
                newPeriod=1;
        }
        
        setRtm(newPeriod);
        
//        std::cout<<oldPeriod<<" >> "<<oldStimulus<<" -> "<<newStimulus<<" >> "<<this->getRtm()<<"\n";
        
        oldStimulus=newStimulus;
        oldPeriod=this->getRtm();
    }
    //funzione di aggiornamento della frequenza con legge di Weber
    //  gli ultimi 2 parametri sono fittizi servono solo per evitare di modificare le chiamate a funzione
    //ATTENZIONE: qui il ritmo non viene integrato con la magnitudine
    void updateRtm_weber(double newStimulus, double fake1, double fake2){
        
        //double newPeriod=0.01;
        double newPeriod=1;
        double newRelevance;
        
        //k neutra per prova
        k_weber=1;
        
        if(oldStimulus!=0) {
//            if( (oldFeature - newFeature) >= 0 )
//                newDelta = ( (oldFeature - newFeature) / oldFeature ) / oldRtm;
//            else
//                newDelta = oldRtm / ( (oldFeature - newFeature) / oldFeature );
            
            //newRelevance = k_weber * ( (oldStimulus - newStimulus) / oldStimulus);
            
            newRelevance = k_weber * ( (oldStimulus - newStimulus) / newStimulus);
            
            newPeriod = oldPeriod - newRelevance;

            if (newPeriod < 0.01)
                newPeriod=0.01;
            else if (newPeriod > 1)
                newPeriod=1;
        }
        
        setRtm(newPeriod);
        
//        std::cout<<oldPeriod<<" >> "<<oldStimulus<<" -> "<<newStimulus<<" >> "<<this->getRtm()<<"\n";
        
        oldStimulus=newStimulus;
        oldPeriod=this->getRtm();
    }
    double getRtm(){
        return behaviourRtm;
    }
    void setDefaultRtm(){
        
        double newRtm=1;
        
        updated=true;
        
        if(getMagnitude()!=0)
            newRtm=1/getMagnitude();
        
        
        int trun=(int) (newRtm*PRECISION);
        behaviourRtm=((double)trun)/PRECISION;
        
        pthread_mutex_lock(&memMutex);
        
        if(dead()) return;
        
        //comunica alla WM lo stato del ritmo (per eventuali stampe)
        std::vector<WM_node*> app=WM->getNodesByInstance(getInstance());
        for(int i=0;i<app.size();i++)
            app[i]->rtm=behaviourRtm;
        
        pthread_mutex_unlock(&memMutex);
    }
    void setRtm(double newRtm){
        
        updated=true;
        
        int trun=(int) (newRtm*PRECISION);
        behaviourRtm=((double)trun)/PRECISION;
        
        //comunica alla WM lo stato del ritmo (per eventuali stampe)
        std::vector<WM_node*> app=WM->getNodesByInstance(getInstance());
        for(int i=0;i<app.size();i++)
            app[i]->rtm=behaviourRtm;
        
        
        //quando si è in quiescenza non aspettare 2 secondi!!
        if(behaviourRtm == QUIESCENCE)
            behaviourRtm = DEFAULT;
    }
    double getMagnitude(){
        return behaviourMagnitude;
    }
    void setMagnitude(double newMagnitude){
        behaviourMagnitude=newMagnitude;
    }
    //setta il nome del behaviour
    void setName(std::string newName){
        behaviourName=newName;
    }
    //restituisce il nome del behaviour, compreso di parametri
    std::string getName(){
        return behaviourName;
    }
    //setta l'istanza del behaviour (nome+parametri)
    void setInstance(std::string newInstance){
        behaviourInstance=newInstance;
    }
    //restituisce l'istanza del behaviour, compreso di parametri
    std::string getInstance(){
        return behaviourInstance;
    }
    void setReleaser(bool r){
        releaser=r;
    }
    bool getReleaser(){
        return releaser;
    }
    double getStimulus(){
        return oldStimulus;
    }
    double setStimulus(double s){
        oldStimulus=s;
    }
    bool isUpdated(){
        return updated;
    }
    /**
     * 
     * @param instances vettore delle istanze da amplificare
     * 
     */
    void amplifyAllInstances(std::vector<WM_node *> instances){
        //per ogni istanza
        for(int i=0;i<instances.size();i++)
            //se l'istanza è rilasciata
            if(instances[i]->isBranchReleased()){
                //incrementane il contatore dei goal raggiunti
                instances[i]->goalCount++;
                //radice del sottoalbero da amplificare
                (instances[i]->father)->amplify(instances[i]->magnitude);
            }
    }

    
    /**
     * funzione di modifica delle variabili in ambito protetto
     * può essere utilizzata per modificare i valori degli attuatori
     * se la risorsa è disponibile il valore viene inviato (sommandolo
     * ad altri se necessario), altrimenti la funzione non ha effetto.
     * 
     * @param receiver: istanza del behaviour da contattare
     *  se NULL il sistema considera la variabile da modificare
     *  come non condivisa (modificandola direttamete)
     * @param var: nome della variabile di sistema da modificare
     * @param value: valore da inserire nella variabile
     * 
     * @return 1 se il valore è stato inviato, -1 se è stato parzialmente inviato
     *  (ie. non è stato il primo a scrivere), 0 altrimenti
     */
    template<class T>
    int send(std::string receiver, std::string var, T value){
        //se il ricevente è nullo la variabile non è condivisa
        if(receiver==""){
            //modifica la variabile direttamente ed esci
            WMV.set<T>(var,value);
            return 1;
        }
        double double_type,count;
        int out=1;
        //recupera il nodo che corrisponde al ricevitore
        std::vector< WM_node*> recNode=WM->getNodesByInstance(receiver);
        //recupera la lista dei nodi vincitore
        std::vector< WM_node*> winnerNodes = WM->getNodesByInstance(recNode[0]->winner);
        
        //se il ricevitore non è istanziato esci senza inviare
        if(recNode.size()==0) return 0;
        
//        std::cout<<"sending, "<<this->getInstance()<<", "<<this->getRtm()<<"\n";
        
        //se il vinctore non esiste (ie. ricevitore libero o vincitore rimosso)
        if(winnerNodes.size()==0 || 
                //oppure sono io il vincitore
                winnerNodes[0]->instance == this->getInstance() ||
                //oppure sono meglio del vincitore
                winnerNodes[0]->rtm>this->getRtm() ||
                (this->getRtm()==0.01 && this->getMagnitude() > WM->getInstanceMagnitude(winnerNodes[0]->instance) ) ){
            
            //modifica la variabile
            count = WMV.get<double>(var+".count");
            
            //se il vincitore non è vuoto e non sono io (ie. ho appena vinto la competizione)
            if( winnerNodes.size() != 0 && winnerNodes[0]->instance!=this->getInstance() ) {
                //se il tipo T è double
                if(typeid(T) == typeid(double_type)){
                    //calcola la media dei valori
                    T old = WMV.get<T>(var);
                    T sum = old + value;
                    WMV.set<T>(var, sum);
                }
                //altrimenti
                else
                    //setta la nuova variabile
                    WMV.set<T>(var, value);
                
                //ritorna -1 perchè il valore è parzialmente scritto
                out=-1;
            }
            //altrimenti (ie. sono il primo a scrivere oppure continuo ad essere il vincitore)
            else{
                //modifica direttamente la variabile
                WMV.set<T>(var, value);
                count=0;
            }
            //aumenta il contatore della variabile
            count++;
            WMV.set<double>(var+".count", count);
            
            //occupa la risorsa
            for(int i=0;i<recNode.size();i++){
                
                if(recNode[i]->winner!=this->getInstance())
                    recNode[i]->oldWinner=recNode[i]->winner;
                
                recNode[i]->winner=this->getInstance();
                recNode[i]->winnerRtm=this->getRtm();
                recNode[i]->winnerMag=this->getMagnitude();
            }
        }
//            std::cout<<this->getInstance()<<" "<<var<<"="<<value<<" count:"<<count<<"\n";
            
        //atrimenti esco senza scrivere
        else return 0;
        
        //std::cout<<this->getInstance()<<" sended "<<value<<" to "<<receiver<<"\n";
        return out;
    }
    bool isFreezed(){
        bool result=false;
        pthread_mutex_lock(&memMutex);
        
        if(WM->isReleased("freeze("+this->getInstance()+")"))
            result=true;
        //questo ELSE è stato aggiunto per bloccare i sottoschemi di PLAN
        else {
            int i=0;
            //finche ci sono schemi e sono freezzati
            while(i<WM->getNodesByInstance(this->getInstance()).size() && WM->getNodesByInstance(this->getInstance())[i]->freezed)
                //incrementa
                i++;
            //se ho controllato tutti i figli (ie. erano tutti freezzati)
            if(i==WM->getNodesByInstance(this->getInstance()).size())
                //allora sono freezzato
                result=true;
        }
        
        pthread_mutex_unlock(&memMutex);
    }
    
    /** funzione di settaggio del releaser interno nei nodi della WM
     * 
     * @param ir: releaser interno del processo, viene dato in output
     *      dallo schema percettivo.
     * 
     * NB. prima di eseguire questa funzione va lockato il semaforo della
     *      WM.
     */
    void setNodeInternalReleaser(bool ir){
        for(int i=0; i<myNodeList.size(); i++){
            myNodeList[i]->internalReleaser=ir;
        }
    }
    
    
    /** funzione di controllo del behaviour in WM
     * 
     * @return TRUE se il behaviour è presente in WM
     * 
     * inoltre, setta il behaviourInstance e behaviourRtm sulla base
     * del nodo eventualmente trovato
     */
    bool perceptWM(){
        std::vector<WM_node*> me;
        bool result=false;
        
        pthread_mutex_lock(&memMutex);
        
        if(WM==NULL){
            pthread_mutex_unlock(&memMutex);
            return false;
        }
        //controlla se sono in memoria
        me = WM->getNodesByInstance(getInstance());
        myNodeList = me;
        
        //se sono in memoria
        if(me.size()!=0){
            //ritorna true
            result=true;
            
            //resetta la prelazione per tutte le mie istanze
//            for(int i=0;i<me.size();i++)
//                me[i]->winnerRtm=0;
            
            //il controllo è su una delle istanze
            
            //se il releaser è disattivo
            if(!WM->isReleased(getInstance()) || 
                    //oppure sono teleologico ed ho raggiunto il goal
                    (me[0]->teleological && me[0]->goalStatus())){
                //inibisci lo schema motorio
                setRtm(QUIESCENCE);
                setReleaser(false);
            }
            //altrimenti sono rilasciato
            else
            {
                setMagnitude(WM->getInstanceMagnitude(getInstance()));
                
//                if(oldPeriod==0)
//                    setRtm(DEFAULT);
//                else
//                    setRtm(oldPeriod);
                
                setReleaser(true);
            }
            
        }
        
        //inizializza a false, verrà messa a true se qualcuno chiama updateRtm
        this->updated=false;
        
        pthread_mutex_unlock(&memMutex);
        //altrimenti NON sono in memoria e ritorna false       
        return result;
    }
private:
    std::string behaviourName;
    std::string behaviourInstance;
    double behaviourRtm;
    double behaviourMagnitude;
    bool releaser;
    //true se il rtm è stato aggiornato
    bool updated;
    //lista dei nodi associati al processo
    std::vector< WM_node * > myNodeList;
    
    //updateRtm Weber law
    double oldStimulus;
    double oldPeriod;
    double k_weber;
};

class AliveBehaviour : public Behaviour{
public:
    AliveBehaviour(std::string name){
        setName(name);
        setInstance(name);
        setRtm(QUIESCENCE);
    }
    bool perceptualSchema(){
        return true;
    }
    void motorSchema(){
        WM_node* expNode;
        std::string goalString;
        EC_ref semanticFromEclipse,goalFromEclipse;
        EC_ref schemaInstance;
        EC_word goal,goalList;
        
        pthread_mutex_lock(&memMutex);
        
        if( dead() )
            return;
        //per adesso si mette un ritmo di default
        setRtm(0.1);
        
        //amplifica gli eventuali nodi
        WM->amplifyNodes();
        
        //se esiste un nodo da espandere
        if((expNode = WM->getExpandableNode()) != NULL) {
            //espandi il nodo (motor schema)
            //std::cout<<"EXPAND: "<<expNode->instance<<"\n";
            if(expNode->father!=NULL){
                expNode->magnitude=((expNode->father->magnitude)*(expNode->ltMagnitude))/expNode->father->son.size();
                //expNode->amplification=(expNode->father->amplification)/expNode->father->son.size();
            }
            
            //richiedi ad ECLIPSE la definizione semantica dello schema
            goalString="getSemantics("+(expNode->instance)+")";
            post_goal(goalString.c_str());
            EC_resume();
            
            //se la risposta non è una definizione semantica
            if(EC_resume(EC_atom("ok"),semanticFromEclipse) != EC_yield)
            {
                //rimuovi il nodo
                std::cout<<"SYSTEM: no semantic for "<<expNode->instance<<"\n";
                remove(expNode);
            }
            //altrimenti espando il nodo
            else
            {
                semanticFromEclipse.cut_to();
                //se non vi sono altre istanze sveglie
                if (!(WM->isAwake(expNode->instance)))
                    //prova ad allocare il thread
                    if(wakeUp(expNode->name, expNode->instance))
                        //se lo hai allocato allora è concreto
                        expNode->abstract = false;
                    else
                        //altrimenti è astratto
                        expNode->abstract = true;
                else
                    //altrimenti (ie. esiste una istanza concreta sveglia) è concreto
                    expNode->abstract=false;
                
                //altrimenti carica la semantica dello schema nella WM
                if(EC_word(semanticFromEclipse).is_nil()){
                    loadSemantics(EC_word(semanticFromEclipse),expNode);
                }
                
                
                //congeda ECLIPSE
                EC_resume();
                
                //richiedi ad ECLIPSE il goal dell'istanza
                goalString="getGoal("+(expNode->instance)+")";
                post_goal(goalString.c_str());
                EC_resume();
                
                //se esiste un goal per l'istanza
                if(EC_resume(EC_atom("ok"),goalFromEclipse) == EC_yield)
                {
                    expNode->teleological=true;
                    goalList=EC_word(goalFromEclipse);
                    //estrai i vari elementi del goal
                    while(goalList.is_nil())
                    {
                        goalList.is_list(goal,goalList);
                        //std::cout<<"coal: "<<functor2string(goal)<<"\n";
                        expNode->goal.push_back(functor2string(goal));
                    }
                }
                else
                    expNode->teleological=false;
                
                //congeda ECLIPSE
                EC_resume();
                
                expNode->expanded=true;
		WMV.set<std::string>("alive", expNode->instance);
            }
            
//            std::cout<<"...DONE\n";
        }
        
        //aggiorna la magnitudine dei nodi dopo l'inserimento
        WM->updateMagnitude();
        pthread_mutex_unlock(&memMutex);
    }

    void start(){
        std::cout<<"SYSTEM: hello world!\n";
    }
    void exit(){
        std::cout<<"SYSTEM: bye\n";
        ec_cleanup();
    }
};

#endif	/* SEED_HEADER_H */

