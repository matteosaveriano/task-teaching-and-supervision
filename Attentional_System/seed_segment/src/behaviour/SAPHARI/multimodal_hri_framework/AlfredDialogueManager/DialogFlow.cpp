/* 
 * File:   DialogFlow.cpp
 * Author: lorenzo
 * 
 *   
 *
 * Created on 17 settembre 2012, 12.13
 */

#include "DialogFlow.h"


        
DialogFlow::DialogFlow(int ID,std::string label) {
    this->ID=ID;
    this->label=label;
}
DialogFlow::DialogFlow() {
    
}


DialogFlow::~DialogFlow() {
}

void DialogFlow::SetLabel(std::string label) {
    this->label = label;
}

std::string DialogFlow::GetLabel() const {
    return label;
}

void DialogFlow::SetID(int ID) {
    this->ID = ID;
}

int DialogFlow::GetID() const {
    return ID;
}

