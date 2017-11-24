/* 
 * File:   POMDPstate.h
 * Author: hargalaten
 *
 * Created on 11 aprile 2013, 9.12
 */

#ifndef POMDPSTATE_H
#define	POMDPSTATE_H

#include <string>
//using namespace std;

class POMDPState {
public:
    POMDPState();
    POMDPState(std::string DF,int FS,std::string userAction,double p);
    virtual ~POMDPState();
    void SetUserAction(std::string action);
    std::string GetUserAction() const;
    void SetFlowState(int flowState);
    int GetFlowState() const;
    void SetDialogFlow(std::string dialogFlow);
    std::string GetDialogFlow() const;

    void SetP(double p);
    double GetP() const;

private:
    std::string dialogFlow;
    int flowState;
    std::string userAction;
    double p;

};

#endif	/* POMDPSTATE_H */

