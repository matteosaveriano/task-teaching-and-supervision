#include "../seed_header.h"

class LTMBehaviour : public Behaviour{
public:
    void exit(){
    }
    void start(){
    }
protected:
};


/*
*   set a WMV to a specific value:
*       set(varName,varValue,rtm)
*           varName: name of the WMV to set\
*           varValue: desired value (bool string or double)
*           rtm: period-rate of setting (only once if 0)
*/
class postGoalBehaviour : public LTMBehaviour{
    public:
    postGoalBehaviour(std::string instance){
      setName(instance2vector(instance)[0]);
      setInstance(instance);
      setRtm(QUIESCENCE);

      goal = instance2vector(instance)[1];
    }

    bool perceptualSchema(){
      return true;
      }
    void motorSchema(){
        EC_ref fromEclipse;

        pthread_mutex_lock(&memMutex);

        post_goal(goal.c_str());
        EC_resume();

        //se la risposta non è una definizione semantica
//        if(EC_resume(/*EC_atom("ok"),*/fromEclipse) != EC_succeed)
//            std::cout<<"postGoal: no response for "<<goal<<"\n";
//        else
//            std::cout<<"postGoal: "<< functor2string(EC_word(fromEclipse)) <<"\n";

        remove(WM->getNodesByInstance(this->getInstance())[0]);

        pthread_mutex_unlock(&memMutex);
    }
protected:
    std::string goal;
};
