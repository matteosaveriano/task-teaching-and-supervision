/* 
 * File:   POMDP.h
 * Author: lorenzo
 *
 * Created on 17 settembre 2012, 11.12
 * Questa classe rappresenta l'intero POMDP.
 * Questa classe estende la classe QObject in modo da usare il sistema 
 * signals&slot.
 */

#ifndef POMDP_H
#define	POMDP_H

#include "Param.h"
#include "tinyxml2.h"
#include "Observ.h"
#include "POMDPState.h"
#include "DialogFlow.h"
#include <iostream>
#include <stdlib.h>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <math.h>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <boost/tuple/tuple_io.hpp>

#include <set>

#include <stack>


using std::stack;
using std::vector;
using namespace tinyxml2;
using namespace boost;
typedef tuple<double, double, double> SummaryState;
typedef vector<POMDPState> Belief;
typedef tuple<Belief, Belief, std::string> Backup;

class POMDP {
    //friend class AlfredDMBehaviour;
public:
    /*Il costruttore prende in input il numero di file DF.xml,i nomi dei file*/
    POMDP(int argc, char** argv);
    /*Carica da un file politica gli elementi di basket e le relative azioni*/
    bool loadPolicy(std::string xmlFile);

    virtual ~POMDP();
    
    void RestoreBS();



   
   
    /*------------------------------Funzioni legate al run---------*/
    /*Aggiorna il belief state*/
    void updateBelieves(vector<Observ> Observation);
    /*Seleziona un'azione*/
    std::string selectAction(std::string TypePolicy);
    /*Esegui Azione*/
    std::string performAction(std::string TypePolicy = "greedy");
    /*Cancella un'ipotesi*/
    void deleteHyp(std::string Hyp);

    /*Ritorna una coppia formata dalle due azioni macchina relative alle due ipotesi migliori di BS*/
    std::pair<std::string, std::string> twoTopHypActions(vector<POMDPState> BS);
    /*Ritorna le due azioni utente (osservazioni) relative alle due ipotesi migliori di BS*/
    std::pair<std::string, std::string> twoTopHypUserAct(vector<POMDPState> BS);
    /*Ritorna gli indici delle due ipotesi maggiori in BS. Se la seconda ipotesi
     non c'è, l'intero sarà -1*/
    std::pair<int, int> twoTopHyp(vector<POMDPState> BS);
    /*-------------------------------------------------------------*/

    /*------------------Funzioni legate alla politica-----------*/
    /*Ritorna il reward atteso eseguendo action*/
    double reward(vector<POMDPState> BS, std::string action);

    /*Restituisce il puntatore al rappresentante nel SummarySpace di BS*/
    SummaryState Summarize(vector<POMDPState> BS);

    /*Decodifica l'azione proveniente dal SummarySpace in una del POMDP classico.
     Am è un'azione Summary.Appartine a MAchineAction.
     BS è il Belief Point su cui applicare Am*/
    std::string decodeAction(std::vector<POMDPState> BS, std::string Am);

    /*Restituisce la distanza tra due punti del SummarySpace. Max=sqrt(3) min=0*/
    double distance(SummaryState b1, SummaryState b2);

    /*Seleziona un'azione tra quelle in MAchineAction*/
    std::string selectAction();

    /*Ritorna l'indice del SBS appartenenta a basket più vicino a b*/
    int closestPoint(SummaryState b, vector<SummaryState> Basket);
    /*----------------------------------------------------------*/





    /*------------------Attributi----------------------------*/
    /*Ultima azione conclusa*/
    std::string lastMA;
    /*Vettore contenete le ipotesi.Rappresenta il Belief state*/
    Belief newBelieves;
    /*Rappresenta il Belief state del passo t-1*/
    Belief oldBelieves;
    /*Una mappa associativa in cui sono memorizzati i DF. La chiava è l'etichetta
     del DF*/
    std::map<std::string, DialogFlow> dialogFlows;

    //Matrice di adiacenza <DF1,FS1,Obs,AM> <DF2,FS2> -> P(<DF2,FS2>\<DF1,FS1,AM>)
    std::map < tuple <std::string, int, std::string, std::string>, std::map<std::pair<std::string, int >, double> > adjMat;

    /*Backups*/
    stack<Backup> storedBS;

    /*Basket di Punti scelti per l'ottimizzazione e le relative azioni*/
    vector <SummaryState> SummaryPoints;
    vector <std::string> optimalAction;

    /*Vettore delle azioni macchina possibili nel Summary State*/
    vector<std::string> MachineAction;
    /*Vettore di insiemi di azioni confondibili*/
    vector<std::set<std::string> > confusione;


};

#endif	/* POMDP_H */