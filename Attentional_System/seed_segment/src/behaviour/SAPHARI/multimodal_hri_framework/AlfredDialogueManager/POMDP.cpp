/* 
 * File:   POMDP.cpp
 * Author: lorenzo
 * 
 * Created on 17 settembre 2012, 11.12
 * 
 * 
 * 
 * 
 */



#include "POMDP.h"

using namespace tinyxml2;

POMDP::POMDP(int argc, char** argv) {
    
    /*Azioni macchina disponibili nel summary space*/
    MachineAction.push_back("Do_act");
    MachineAction.push_back("Request");
    MachineAction.push_back("ChoiceAmong2");
    
    /*Definizione degli insiemi di gesti confondibili*/
    std::set<std::string> conf;
    conf.insert("Avvicinati");
    conf.insert("Vieni_qui");
    conf.insert("Mostra");
    conf.insert("Prendi");
    conf.insert("Dammi");
    conf.insert("Posa");
    confusione.push_back(conf);
    conf.clear();
    conf.insert("Indica");
    conf.insert("Cerca");
    conf.insert("Cerca_Gialla");
    conf.insert("Cerca_Rossa");
    confusione.push_back(conf);
    conf.clear();
    conf.insert("Idle");
    conf.insert("Cammina");
    confusione.push_back(conf);
    
    
    XMLDocument* doc;

    XMLElement* tempNode; //memorizzo temporaneamente i nodi e la root
    XMLElement* tempObs; //memorizzo temporaneamente Observ
    XMLElement* tempMA; //memorizzo temporaneamente MachineAction
    XMLElement* tempTrans; //memorizzo temporaneamente Link_to

    FlowState newState; //memorizzo nuovi nodi del grafo
    DialogFlow newDF; //memorizzo nuovi DF
    POMDPState InitState;

#warning "Controlla da dove parte"

    for (int i = 0; i < argc; i++) {
        
        //leggo le informazioni sul dialogo
        doc = new XMLDocument();
        std::cout<<argv[i]<<"\n";
        if (doc->LoadFile(argv[i]) != 0) throw "xmlFile error";
        
        tempNode = doc->RootElement();
        //Creo l'elemento DialogFlow
        newDF=DialogFlow(i-1,tempNode->Attribute("label"));
        //leggo il primo Node 
        tempNode = tempNode->FirstChildElement();
        
        while (tempNode != NULL) {
            //Creo un elemento FlowState
            newState=FlowState(boost::lexical_cast<int>(tempNode->Attribute("ID")),std::string(tempNode->Attribute("label")));
            //Se è un nodo di Start lo inserisco nel Belief State
            if (tempNode->Attribute("startingP") != NULL) {
                InitState.SetDialogFlow(newDF.GetLabel());
                InitState.SetFlowState(newState.GetID());
                InitState.SetUserAction("StartUp");
                InitState.SetP(boost::lexical_cast<double>(tempNode->Attribute("startingP")));
                newBelieves.push_back(InitState);
            }
            //Inserisco le info sulle osservazioni
            tempObs = tempNode->FirstChildElement();
            while (tempObs != NULL) {
                //Aggiungo l'osservazione alla mappa UserActionModel
                newState.UserActionModel[tempObs->Attribute("name")] = boost::lexical_cast<double>(tempObs->Attribute("P"));
                //Inserisco info sulle Machine Action
                tempMA = tempObs->FirstChildElement();
                //Il while è inutile visto che ogni osservazione ha solo un'azione associata
                while (tempMA != NULL) {
                    newState.MachineAction[tempObs->Attribute("name")] = tempMA->Attribute("name");
                    //Inserisco info sulle transizioni dovute alle azioni
                    //P(s'\am,s)=sum am P(s'\am)sum o P(am\o)P(o)
                    tempTrans = tempMA->FirstChildElement();
                    while (tempTrans != NULL) {
                        if (strcmp(tempTrans->Name(), "Link_To") == 0) {
                            adjMat[tuple <std::string, int, std::string, std::string > (newDF.GetLabel(), newState.GetID(), tempObs->Attribute("name"), tempMA->Attribute("name"))].operator [](std::pair <std::string, int> (newDF.GetLabel(), boost::lexical_cast<int>(tempTrans->Attribute("ID")))) = boost::lexical_cast<double>(tempTrans->GetText());
                        } else
                            adjMat[tuple <std::string, int, std::string, std::string > (newDF.GetLabel(), newState.GetID(), tempObs->Attribute("name"), tempMA->Attribute("name"))].operator [](std::pair <std::string, int> (tempTrans->Attribute("DF"), boost::lexical_cast<int>(tempTrans->Attribute("ID")))) = boost::lexical_cast<double>(tempTrans->GetText());
                        tempTrans = tempTrans->NextSiblingElement();
                    }
                    tempMA = tempMA->NextSiblingElement();
                }
                tempObs = tempObs->NextSiblingElement();
            }
            // Aggiungo  il nuovo FlowState al DF
            newDF.states.push_back(newState);
            tempNode = tempNode->NextSiblingElement();
        }
        
        //Aggiungo il DF al vettore in POMDP
        dialogFlows[newDF.GetLabel()] = newDF;
        delete doc;
    }
    
#warning "Di default i nodi di partenza sono equiprobabili"
    
    double n=newBelieves.size();
    for(vector<POMDPState>::iterator it=newBelieves.begin();it!=newBelieves.end();it++){
        it->SetP(1.0/n);
    }
    lastMA="Greet";
}

POMDP::~POMDP() {
}


/*--------------------------Funzioni di esecuzione------------------*/

bool POMDP::loadPolicy(std::string xmlFile){
    XMLDocument doc;
    if(doc.LoadFile(xmlFile.c_str())!=0) return false;
    XMLElement* SummaryB;
    SummaryState SS;
    
    SummaryB=doc.FirstChildElement();
    while(SummaryB!=NULL){
        SS.get<0>()=lexical_cast<double>(SummaryB->FirstChildElement("First")->GetText());
        SS.get<1>()=lexical_cast<double>(SummaryB->FirstChildElement("Second")->GetText());
        SS.get<2>()=lexical_cast<double>(SummaryB->FirstChildElement("Comp")->GetText());
        SummaryPoints.push_back(SS);
        optimalAction.push_back(SummaryB->FirstChildElement("Action")->GetText());    
        
        SummaryB=SummaryB->NextSiblingElement();
        }
         
    return true;
}

void POMDP::updateBelieves(vector<Observ> Observation) {
    if(newBelieves.size()< 1) throw "updateBelief error";
    
    //Normalizza BS
    double norm=0.0;
    for(int i=0;i<newBelieves.size();i++){
        norm+=newBelieves[i].GetP();
    }
    for(int i=0;i<newBelieves.size();i++){
        newBelieves[i].SetP(newBelieves[i].GetP()/norm);
    }
    
    
//    //Se mi trovo in ChoiceAmong 2, devo codificare le 2 ipotesi top con prima e seconda
//    if((*newBelieves)[0]->GetDialogFlow().compare("ChoiceAmong2")==0){
//        pair<string,string > topHyp=twoTopHypUserAct(backUpNew);
//        for(int i=0;i<NBestObs->size();i++){
//            if((*NBestObs)[i]->GetCommand().compare(topHyp.first)==0)
//                (*NBestObs)[i]->SetCommand("Prima");
//            else if((*NBestObs)[i]->GetCommand().compare(topHyp.second)==0)
//                    (*NBestObs)[i]->SetCommand("Seconda");
//            else if ((*NBestObs)[i]->GetCommand().compare("No")==0)
//                    (*NBestObs)[i]->SetCommand("Nessuna");
//        }
//    }
  
    /*Tupla composta da DF,FS,UA,MA*/
    tuple<std::string, int, std::string, std::string> belPoint;
    
    /*P(s'/s)*/
    std::map<std::pair<std::string, int>, double> stateTrans;
    
    //Iteratori
    std::map<std::pair<std::string, int>, double>::iterator stateTransIterator;
    std::map<std::pair<std::string, int >, double>::iterator successors;
    
    //Calcolo P(s'/s)
    for (unsigned int i = 0; i < newBelieves.size(); i++) {
        belPoint.get<0>() = newBelieves[i].GetDialogFlow();
        belPoint.get<1>()=newBelieves[i].GetFlowState();
        belPoint.get<2>()=newBelieves[i].GetUserAction();
        belPoint.get<3>()=lastMA;
        //Controllo se ci sono successori 
        if (adjMat.count(belPoint) != 0) {
            //Scorro i successori
            for (successors = adjMat[belPoint].begin(); successors != adjMat[belPoint].end(); successors++) {
                //P(s')+=P(s'\s)*P(s)
                stateTrans[successors->first] += successors->second * newBelieves[i].GetP();
            }
        }
        
        /*Caso speciale in cui la transizione ai prossimi stati avviene con qualunque azione macchina ("*" in machine action)*/
        belPoint.get<3>()="*";
        if (adjMat.count(belPoint) != 0) {
            //Scorro i successori
            for (successors = adjMat[belPoint].begin(); successors != adjMat[belPoint].end(); successors++) {
                //P(s')+=P(s'\s)*P(s)
                stateTrans[successors->first] += successors->second * newBelieves[i].GetP();
            }
        }
    }
    //Salvo il vecchio belief
    oldBelieves = newBelieves;
    newBelieves= Belief();
    for (unsigned int i = 0; i < Observation.size(); i++) {
        for (stateTransIterator = stateTrans.begin(); stateTransIterator != stateTrans.end(); stateTransIterator++) {
            //Se nello stato è visibile l'azione utente creo un nuovo BFPoint
            if (dialogFlows[stateTransIterator->first.first].states[stateTransIterator->first.second].UserActionModel.count(Observation[i].GetCommand()) != 0) {
                /*La probabilità del nuovo stato P(s')+=P(Obs\a) P(a\s) P(s) */
                newBelieves.push_back( POMDPState(stateTransIterator->first.first, stateTransIterator->first.second, Observation[i].GetCommand(), Observation[i].GetP() * stateTransIterator->second * dialogFlows[stateTransIterator->first.first].states[stateTransIterator->first.second].UserActionModel[Observation[i].GetCommand()]));
            }
        }

    }
    
}

std::string POMDP::selectAction(std::string TypePolicy) {
    if(TypePolicy=="greedy"){
        std::pair<std::string,std::string> TopHypAct=twoTopHypActions(newBelieves);
        return TopHypAct.first;       
    }
    
    else if(TypePolicy=="optim"){
        SummaryState SState=Summarize(newBelieves);
  
        int closest=closestPoint(SState,SummaryPoints);
        return decodeAction(newBelieves,optimalAction[closest]);
   
        /*fatta a mano*/
//        pair <string,string> Top=twoTopHypActions(newBelieves);
//        if(SState->get < 2 > ()>1.5 ||SState->get < 0 > ()>0.8)
//            return decodeAction(*newBelieves,"Do_act");
//        else if(SState->get < 0 > ()>0.7 && SState->get < 2 > ()>1)
//            return decodeAction(*newBelieves,"Request");
//        else if (Top.first!=Top.second)
//            return decodeAction(*newBelieves,"ChoiceAmong2");
//        else return decodeAction(*newBelieves,"Request");
        
//         pair <string,string> Top=twoTopHypActions(newBelieves);
//        if(SState->get < 2 > ()>1.0 ||SState->get < 0 > ()>0.6)
//            return decodeAction(*newBelieves,"Do_act");
//        else if(SState->get < 0 > ()>0.4 && SState->get < 2 > ()>1)
//            return decodeAction(*newBelieves,"Request");
//        else if (Top.first!=Top.second)
//            return decodeAction(*newBelieves,"ChoiceAmong2");
//        else return decodeAction(*newBelieves,"Request");
        /*Greedy*/
//        double rew=-10000000000000;
//        int act;
//        for(int i=0;i<MachineAction.size();i++){
//        if(reward(*newBelieves,MachineAction[i])>rew){
//            rew=reward(*newBelieves,MachineAction[i]);
//            act=i;
//        }
//        }
//        return decodeAction(*newBelieves,MachineAction[act]);
    
    
    }
        
}

double POMDP::reward(vector<POMDPState> BS, std::string action) {
    if (BS.size() == 0) throw "Reward riceve BS vuoto";
    double rew = 0.0;
    std::string originalAct;
    std::pair<std::string,std::string> TopAct=twoTopHypActions(BS);
    
    originalAct = decodeAction(BS, "Do_act");
    
    if (action == "Do_act") {
        for (int i = 0; i < BS.size(); i++) {
            if (dialogFlows[BS[i].GetDialogFlow()].states[BS[i].GetFlowState()].MachineAction[BS[i].GetUserAction()] == originalAct)
                rew += BS[i].GetP()*10.0;
            else
                rew += BS[i].GetP()*(-20.0);
        }
    } else if (action == "Request") {
        for (int i = 0; i < BS.size(); i++) {
           if (dialogFlows[BS[i].GetDialogFlow()].states[BS[i].GetFlowState()].MachineAction[BS[i].GetUserAction()] == originalAct)
                rew += BS[i].GetP()*-0.5;
            else
                rew += BS[i].GetP()*-3.0;
        }
    } else if (action == "ChoiceAmong2") {
        if(TopAct.second=="NULL") rew=-100000.0;
        
        else if(TopAct.first==TopAct.second)
            rew=-100000.0;
        else
            rew=-1;
    }
    
    return rew;
}
/**/
std::pair<std::string,std::string> POMDP::twoTopHypActions(vector<POMDPState> BS){
   /*Attenzione: la funzione può ritornare due azioni uguali!*/
    
    std::pair<int, int> TopHyp=twoTopHyp(BS);
      
    std::string Am_first=dialogFlows[BS[TopHyp.first].GetDialogFlow()].states[BS[TopHyp.first].GetFlowState()].MachineAction[BS[TopHyp.first].GetUserAction()];
    std::string Am_second="NULL";
    if(TopHyp.second!=-1)
        Am_second=dialogFlows[BS[TopHyp.second].GetDialogFlow()].states[BS[TopHyp.second].GetFlowState()].MachineAction[BS[TopHyp.second].GetUserAction()];

    return std::pair<std::string,std::string>(Am_first,Am_second);
    
    
}
std::pair<std::string,std::string> POMDP::twoTopHypUserAct(std::vector<POMDPState> BS){
   
    std::pair<int,int> TopHyp=twoTopHyp(BS);

    std::string UA_first=BS[TopHyp.first].GetUserAction();
    std::string UA_second="NULL";
    if(TopHyp.second!=-1)
        UA_second=BS[TopHyp.second].GetUserAction();
        
    return std::pair<std::string,std::string>(UA_first,UA_second);
    
    
}
std::pair<int,int> POMDP::twoTopHyp(std::vector<POMDPState> BS){
    int first = -1, second = -1;
    double val_first = 0.0, val_second = 0.0;
    /*Trovo le probabilità delle due maggiori ipotesi*/
    for (unsigned int i = 0; i < BS.size(); i++) {
        if (BS[i].GetP() > val_first) {
            second = first;
            val_second = val_first;
            first = i;
            val_first = BS[i].GetP();
        } else if (BS[i].GetP() > val_second) {
            second = i;
            val_second = BS[i].GetP();
        }
    }
   return std::pair<int,int>(first,second);
      
}


std::string POMDP::performAction(std::string TypePolicy) {
    std::string req;
    if (newBelieves.size() == 0) {
//        std::cout<<"ALFRED: Non ho capito-BS vuoto\n";
        newBelieves = oldBelieves;
        oldBelieves.clear();
        return "unknow";
     } else {
        std::string action = selectAction(TypePolicy);
        std::pair<std::string,std::string> TopHypActions=twoTopHypActions(newBelieves);
        std::string greedyAction=TopHypActions.first;        
        
        if (action == "Request") {
            Backup back(newBelieves,oldBelieves,lastMA);
            storedBS.push(back);
            oldBelieves.clear();
            newBelieves.clear();
            POMDPState b0 ("Request", 0, "-", 1);
            newBelieves.push_back(b0);
            std::string temp="Request-";
            temp.append(greedyAction.c_str());
            return temp;
        }
        else if (action == "ChoiceAmong2") {
            if(TopHypActions.second=="NULL" || TopHypActions.second==TopHypActions.first) 
               throw "Error ChoiceAmong2";
            Backup back(newBelieves,oldBelieves,lastMA);
            storedBS.push(back);
            oldBelieves.clear();
            newBelieves.clear();
            POMDPState b0 ("ChoiceAmong2", 0, "-", 1);
            newBelieves.push_back(b0);
            
            std::string temp="ChAmTw-";
            temp.append(greedyAction.c_str());
            temp.append("-");
            temp.append(TopHypActions.second.c_str());
            return temp;
        
        }
        //Azioni interne alla gestione dei BS
        else if (action == "Deny") {
            Backup back=storedBS.top();
            storedBS.pop();
            newBelieves=back.get<0>();
            oldBelieves=back.get<1>();
            lastMA=back.get<2>();
            deleteHyp(twoTopHypActions(newBelieves).first);
            
            return performAction();
        } else if (action == "Confirm") {
            Backup back=storedBS.top();
            storedBS.pop();
            newBelieves=back.get<0>();
            oldBelieves=back.get<1>();
            lastMA=back.get<2>();
            
            return performAction("greedy");
        }  else if (action == "Deny1Hyp") {
            Backup back=storedBS.top();
            storedBS.pop();
            newBelieves=back.get<0>();
            oldBelieves=back.get<1>();
            lastMA=back.get<2>();
            //Cancello la prima ipotesi
            deleteHyp(TopHypActions.first);
            
            return performAction();
        }  else if (action == "Deny2Hyp") {
            Backup back=storedBS.top();
            storedBS.pop();
            newBelieves=back.get<0>();
            oldBelieves=back.get<1>();
            lastMA=back.get<2>();
            //Cancello le prime due ipotesi
            deleteHyp(TopHypActions.first);
            deleteHyp(TopHypActions.second);
            return performAction();
        }//Azioni di interazione con l'utente
        else {
            //Se sono arrivato qui action contiene l'azione più probabile del belief
            
            Backup back(newBelieves,oldBelieves,lastMA);
            storedBS.push(back);
            oldBelieves.clear();
            newBelieves.clear();
            
            POMDPState b0 = POMDPState("No", 0, "-", 1);
            newBelieves.push_back(b0);
            return greedyAction;

        }
    }
}

void POMDP::RestoreBS() {
    
        Backup back = storedBS.top();
        storedBS.pop();
        newBelieves = back.get < 0 > ();
        oldBelieves = back.get < 1 > ();
        std::pair<std::string,std::string> TopHypAct=twoTopHypActions(newBelieves);
        lastMA = TopHypAct.first;
     }


void POMDP::deleteHyp(std::string action) {
//    std::cout<<"AAAAAAAA:"<<action<<"\n";
    vector<POMDPState>::iterator i;
    /*New version*/
    vector<POMDPState> newBS;
    POMDPState element;
    for (i = newBelieves.begin(); i != newBelieves.end(); i++) {
        if (action != dialogFlows[i->GetDialogFlow()].states[i->GetFlowState()].MachineAction[i->GetUserAction()]) {
            element = *i;
            newBS.push_back(element);

        }
    }
    newBelieves.clear();
    newBelieves = newBS;
    //----------//


}
/*-------------------------------------------------------------------*/




/*---------------------------Funzioni legate alla politica--------------*/
SummaryState POMDP::Summarize(vector<POMDPState> BS) {
    if (BS.size() == 0) throw "Summarize Riceve BS vuoto";
    SummaryState sbs;
    std::pair<int, int> TopHyp = twoTopHyp(BS);
    std::pair<std::string, std::string> TopActs = twoTopHypActions(BS);

    /*Controllo la compatibilità della azione più probabile con le altre*/
    std::string Am_first = TopActs.first;
    if (TopActs.second == "NULL")
        return sbs = SummaryState(BS[TopHyp.first].GetP(), 0.0, 0);
    else if (Am_first == TopActs.second)
        return sbs = SummaryState(BS[TopHyp.first].GetP(), BS[TopHyp.second].GetP(), 2);
    else {
        double comp = 0.0;
        for (int i = 0; i < BS.size(); i++) {
            if (i != TopHyp.first) {
                std::string Am = TopActs.first;
                if (Am == Am_first) {
                    comp += BS[i].GetP();
                }
            }
        }
        comp = comp / (1.0 - BS[TopHyp.first].GetP());
        sbs = SummaryState(BS[TopHyp.first].GetP(), BS[TopHyp.second].GetP(), comp);
        return sbs;
    }
}

std::string POMDP::decodeAction(vector<POMDPState> BS, std::string Am) {
    if (BS.size() == 0) throw "decodeAction riceve BS vuoto";

    bool AmIsValid = false;
    for (unsigned int i = 0; i < MachineAction.size(); i++) {
        if (MachineAction[i] == Am) {
            AmIsValid = true;
            break;
        }
    }
    if (AmIsValid == false) throw "decodeAction riceve Am non valida";

    if (Am == "Do_act") {
        std::pair<std::string, std::string> TopAct = twoTopHypActions(BS);
        return TopAct.first;
    } else
        return Am;
}

double POMDP::distance(SummaryState b1, SummaryState b2) {
    //     return (sqrt(pow(b1.get < 0 > () - b2.get < 0 > (), 2)) + 0.5 * (sqrt(pow(b1.get < 1 > () - b2.get < 1 > (), 2))));
    return sqrt(pow(b1.get < 0 > () - b2.get < 0 > (), 2) + pow(b1.get < 1 > () - b2.get < 1 > (), 2) + pow(b1.get < 2 > () - b2.get < 2 > (), 2));
}

int POMDP::closestPoint(SummaryState b, vector<SummaryState> Basket) {
    if (Basket.size() == 0) throw "Closest Point riceve BAsket vuoto";
    int best;
    double val_best = 100.0;
    double temp_val;
    for (int i = 0; i < Basket.size(); i++) {
        temp_val = distance(b, Basket[i]);
        if (temp_val < val_best) {
            val_best = temp_val;
            best = i;
        }
    }
    return best;
}

/*---------------------------------------------------------------------*/









