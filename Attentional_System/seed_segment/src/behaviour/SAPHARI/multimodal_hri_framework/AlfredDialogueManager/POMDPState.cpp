/* 
 * File:   POMDPState.cpp
 * Author: lorenzo
 * 
 * Created on 17 settembre 2012, 13.43
 */

#include "POMDPState.h"

POMDPState::POMDPState() {
}
POMDPState::POMDPState(std::string DF,int FS,std::string userAction,double p) {
    this->userAction=userAction;
    this->dialogFlow=DF;
    this->flowState=FS;
    this->p=p;
}

POMDPState::~POMDPState() {
}

void POMDPState::SetUserAction(std::string action) {
    this->userAction = action;
}

std::string POMDPState::GetUserAction() const {
    return userAction;
}

void POMDPState::SetFlowState(int flowState) {
    this->flowState = flowState;
}

int POMDPState::GetFlowState() const {
    return flowState;
}

void POMDPState::SetDialogFlow(std::string dialogFlow) {
    this->dialogFlow = dialogFlow;
}

std::string POMDPState::GetDialogFlow() const {
    return dialogFlow;
}
void POMDPState::SetP(double p) {
        this->p = p;
    }

double POMDPState::GetP() const {
        return p;
    }
