/* 
 * File:   FlowState.cpp
 * Author: lorenzo
 * 
 * Created on 17 settembre 2012, 12.15
 */

#include "FlowState.h"

FlowState::FlowState(int ID, std::string label) {
    this->ID=ID;
    this->label=label;
}
FlowState::FlowState() {
    
}

FlowState::~FlowState() {
}

void FlowState::SetLabel(std::string label) {
    this->label = label;
}

std::string FlowState::GetLabel() const {
    return label;
}

void FlowState::SetID(int ID) {
    this->ID = ID;
}

int FlowState::GetID() const {
    return ID;
}

