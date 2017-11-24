/* 
 * File:   FlowState.h
 * Author: lorenzo
 *
 * Created on 17 settembre 2012, 12.15
 * Un Flow State è un nodo di un DF. Rappresenta un stato del dialogo ed è carartterizzato
 * da un ID (intero) e una etichetta (string).
 * La P(o\s) è memorizzata in una struttura di tipo map:o->P(o), dove o è una
 * azione osservabile (stringa) e P(o) la probabilità (double).
 */

#ifndef FLOWSTATE_H
#define	FLOWSTATE_H

#include <map>
#include <string>
#include <vector>

//using namespace std;

class FlowState {
public:
    FlowState();
    FlowState(int ID,std::string label);
    virtual ~FlowState();
    void SetLabel(std::string label);
    std::string GetLabel() const;
    void SetID(int ID);
    int GetID() const;
    
    /*Mappa delle Azioni Utente attese*/
    std::map<std::string, double> UserActionModel;
    /*Mappa osservazioni---> azione da fare */
    std::map<std::string,std::string> MachineAction;

private:
    int ID;
    std::string label;
    
};

#endif	/* FLOWSTATE_H */

