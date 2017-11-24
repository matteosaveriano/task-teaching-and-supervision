/* 
 * File:   DialogFlow.h
 * Author: lorenzo
 *
 * Created on 17 settembre 2012, 12.13
 * 
 * Questa classe rappresenta un flusso di dialogo.
 * Un flusso di dialogo è caratterizzato da una etichetta(string) e un 
 * ID (intero). 
 * Un DF è un grafo i cui nodi sono i FlowState, raccolti una struttura di 
 * tipo vector.
 */

#ifndef DIALOGFLOW_H
#define	DIALOGFLOW_H

#include "FlowState.h"
#include <vector>

//using namespace std;

class DialogFlow {
public:
    DialogFlow(int ID, std::string label);
    DialogFlow();
    virtual ~DialogFlow();
    void SetLabel(std::string label);
    std::string GetLabel() const;
    void SetID(int ID);
    int GetID() const;
    std::vector<FlowState> states;
    
private:
    int ID;
    std::string label;
};

#endif	/* DIALOGFLOW_H */

