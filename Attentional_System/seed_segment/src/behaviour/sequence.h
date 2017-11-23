/* 
 * File:   sequence.h
 * Author: hargalaten
 *
 * Created on 3 luglio 2015, 10.22
 */

#ifndef SEQUENCE_H
#define	SEQUENCE_H

#include "../seed_header.h"


class SequenceBehaviour : public Behaviour {
public:

    SequenceBehaviour(std::string instance) {
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        
        //prendi l'indirizzo della mia istanza
        if(WM!=NULL)
            myAddress = WM->getNodesByInstance(instance)[0];
        
        std::string seqString=instance;
        
        seqString=strSubstitute(seqString,'[','(');
        seqString=strSubstitute(seqString,']',')');
        
        seqString=instance2vector(seqString)[1];
        
        schemaDefinition = instance2vector("$" + seqString);
        
//        for(int i=0; i<schemaDefinition.size();i++){
//            std::cout<<"sd: "<<schemaDefinition[i]<<"\n";
//        }
        
        //step 1 - primo passo della sequenza
        step = 1;
        
        addFromDef(schemaDefinition[step]);
    }

    bool perceptualSchema() {
        goal = false;
        pthread_mutex_lock(&memMutex);

        if (myAddress->amplification > 1)
            myAddress->amplification=1;

        this->setRtm(0.2);
        
        //se ho dei figli
        if (myAddress->son.size() != 0) {
            //se il primo figlio ha raggiunto il goal
            if (myAddress->son[0]->goalStatus()) {
                std::cout << "seq: " << myAddress->son[0]->instance << " done!\n";
                goal=true;
            }
        }

        pthread_mutex_unlock(&memMutex);
        return true;
    }

    void motorSchema() {
        pthread_mutex_lock(&memMutex);

        if (dead()) return;
        //se ho un figlio
        if (goal && myAddress->son.size() != 0) {
            //rimuovilo
            remove(myAddress->son[0]);
            step++;
            if(step<schemaDefinition.size()){
//                std::cout<<"step over: "<<step<<"/"<<schemaDefinition.size()<<"\n";
                addFromDef(schemaDefinition[step]);
            }
        }//altrimenti non ho figli
        else if(myAddress->son.size() == 0) {
            //sequenza finita!
            remove(myAddress);
            std::cout << this->getInstance() << " OVER!\n";
        }

        pthread_mutex_unlock(&memMutex);
    }

    void start() {
    }

    void exit() {
    }
    std::string strSubstitute(std::string str, char oc, char nc){
        std::string result;
        std::stringstream ss(str);
        char c;
        //non saltare gli spazi!
        ss >> std::noskipws;
        //leggi il primo carattere della stringa
        ss>>c;
        //mentre non sei a fine stringa
        while (!ss.eof()) {
            if(c == oc)
                result=result+nc;
            else
                result=result+c;
            //prossimo carattere
            ss>>c;
        }
        return result;
    }
    /**
     * Simula alive aggiungendo un nuovo nodo a partire dalla definizione semantica (string).
     * 
     * @param def: definizione dello schema da allocare: [instance, mag, [rel1,...,relN] ]
     * @return puntatore al nuovo nodo nella WM
     */
    WM_node* addFromDef(std::string def){
        WM_node *newSon=myAddress->addSon(instance2vector(def)[1]);
        newSon->ltMagnitude = atoi(instance2vector(def)[2].c_str());
        std::cout<<"ltmag: " <<newSon->ltMagnitude <<"\n";
        std::vector<std::string> releaser = instance2vector(instance2vector(def)[3]);
        //parte da 1 perchè l'elemento 0 (nomeSchema) non c'è
        for(int i=1; i<releaser.size(); i++){
            newSon->releaser.push_back(releaser[i]);
        }
        return newSon;
    }
private:
    std::vector < std::string > schemaDefinition;
    WM_node* myAddress;
    int step;
    int seqLenght;
    bool goal;
};


#endif	/* SEQUENCE_H */

