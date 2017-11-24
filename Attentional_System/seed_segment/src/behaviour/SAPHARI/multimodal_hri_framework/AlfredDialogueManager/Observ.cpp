/* 
 * File:   Observ.cpp
 * Author: lorenzo
 * 
 * Created on 17 settembre 2012, 11.33
 */

#include "Observ.h"

Observ::Observ(std::string command, double p) {
    this->command=command;
    this->p=p;
}
Observ::Observ() {
    
}

Observ::~Observ() {
}

void Observ::SetP(double p) {
    this->p = p;
}

double Observ::GetP() const {
    return p;
}

void Observ::SetCommand(std::string command) {
    this->command = command;
}

std::string Observ::GetCommand() const {
    return command;
}

