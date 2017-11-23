#include "../seed_header.h"
#include <ctype.h>

class WMVBehaviour : public Behaviour{
public:
    void exit(){
    }
    void start(){
    }
    std::string getStringType(std::string strVal)
    {
        if(isdigit(strVal[0]) && strVal.find(".") != std::string::npos){
            return "double";
        }else if(isdigit(strVal[0])){
            return "int";
        }else if(strVal=="TRUE" || strVal=="FALSE"){
            return "bool";
        }else
            return "string";
    }
    void cond_loop(){
        if(rate==0)
            remove(WM->getNodesByInstance(this->getInstance())[0]);
        else
            setRtm(rate);
    }

protected:
    std::string varname,varvalue,vartype;
    double rate;
};


/*
*   set a WMV to a specific value:
*       set(varName,varValue,rtm)
*           varName: name of the WMV to set\
*           varValue: desired value (bool string or double)
*           rtm: period-rate of setting (only once if 0)
*/
class SetBehaviour : public WMVBehaviour{
    public:
    SetBehaviour(std::string instance){
      setName(instance2vector(instance)[0]);
      setInstance(instance);
      setRtm(QUIESCENCE);

      varname = instance2vector(instance)[1];
      varvalue = instance2vector(instance)[2];
      vartype = getStringType(varvalue);
      std::cout<<"vartype: "<<vartype<<"\n";
      rate = atof(instance2vector(instance)[3].c_str());
    }

    bool perceptualSchema(){
      return true;
      }
    void motorSchema(){
        pthread_mutex_lock(&memMutex);
        if(vartype == "double"){
            WMV.set<double>(varname, atof(varvalue.c_str()) );
        } else if(vartype == "int"){
            WMV.set<double>(varname, atoi(varvalue.c_str()) );
        } else if(vartype == "bool"){
            WMV.set<bool>(varname, varvalue=="TRUE"?true:false );
        } else {
            WMV.set<std::string>(varname, varvalue );
        }

        cond_loop();
        pthread_mutex_unlock(&memMutex);
    }
};


/*
*   send a value to a WMV (suffer of contention):
*       set(varName,varValue,rtm)
*           varName: name of the WMV to contend
*           varValue: desired value (bool string or double)
*           rtm: period-rate of sending (only once if 0)
*/
class SendBehaviour : public WMVBehaviour{
    public:
    SendBehaviour(std::string instance){
      setName(instance2vector(instance)[0]);
      setInstance(instance);
      setRtm(QUIESCENCE);

      varowner = instance2vector(instance)[1];
      varname = instance2vector(instance)[2];
      varvalue = instance2vector(instance)[3];
      vartype = getStringType(varvalue);
      std::cout<<"vartype: "<<vartype<<"\n";
      rate = atof(instance2vector(instance)[4].c_str());
    }

    bool perceptualSchema(){
      return true;
      }
    void motorSchema(){
        pthread_mutex_lock(&memMutex);
        if(vartype == "double"){
            send<double>(varowner, varname, atof(varvalue.c_str()) );
        } else if(vartype == "int"){
            send<double>(varowner, varname, atoi(varvalue.c_str()) );
        } else if(vartype == "bool"){
            send<bool>(varowner, varname, varvalue=="TRUE"?true:false );
        } else {
            send<std::string>(varowner, varname, varvalue );
        }

        cond_loop();
        pthread_mutex_unlock(&memMutex);
    }
protected:
    std::string varowner;
};
