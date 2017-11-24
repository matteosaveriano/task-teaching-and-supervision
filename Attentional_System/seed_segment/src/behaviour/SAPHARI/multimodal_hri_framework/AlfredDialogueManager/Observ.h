/* 
 * File:   Observ.h
 * Author: lorenzo
 *
 * Created on 17 settembre 2012, 11.33
 * 
 * Questa classe rappresenta un ipotesi di azione utente proveniente dal motore di fusione.
 * In essa è presente il comando e la probabilità associato ad esso.
 * Un lista di queste Observ rappresenta la n-best list in input al dialog manager.
 */

#ifndef OBSERV_H
#define	OBSERV_H

#include <string>

//using namespace std;

class Observ {
public:
    Observ(std::string command,double p);
    Observ();
    virtual ~Observ();
    void SetP(double p);
    double GetP() const;
    void SetCommand(std::string command);
    std::string GetCommand() const;
private:
    std::string command;
    double p;
};

#endif	/* OBSERV_H */

