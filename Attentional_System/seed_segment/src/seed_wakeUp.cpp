#define STAGE_BEHAVIOUR               0       //include stage behaviour
#define SAPHARI_BEHAVIOUR             0       //include saphari behaviour
#define SAPHARI_HRI_BEHAVIOUR         1       //include saphari multimodal_hri_framework behaviour
#define SAPHARI_SIMULATION            0       //include saphari simulation bahviour
#define SHERPA_BEHAVIOUR		      0       //include sherpa bahviour
#define MOVEIT_BEHAVIOUR		      0       //include moveit bahviour
#define RODYMAN_BEHAVIOUR		      0       //include rodyman bahviour
#define TUM_KUKA_BEHAVIOUR            1       //include behaviour for the TUM kuka robot
#define SHOW_BEHAVIOUR                1       //include show behaviour (needs GRAPHVIZ)

//seed_header provide compilation flags too
#include "seed_header.h"

//---include single behaviour
#include "behaviour/Forget.h"
#include "behaviour/Listing.h"
#include "behaviour/Fg.h"
#include "behaviour/input_stream.h"
#include "behaviour/remember.h"
#include "behaviour/sequence.h"
#include "behaviour/wmv_stream.h"
#include "behaviour/ltm_stream.h"
//---

//---include show behavior
#if SHOW_BEHAVIOUR
  #include "behaviour/show.h"
#endif
//---

//---include kuka behavior
#if TUM_KUKA_BEHAVIOUR
  #include "behaviour/TUM/TUM_kuka.h"
#endif
//---

//---
#if STAGE_BEHAVIOUR
  #include "behaviour/stage/goto.h"
  #include "behaviour/stage/motor.h"
  #include "behaviour/stage/laser_scanner.h"
  #include "behaviour/stage/avoid.h"
#endif
//---

//---
#if MOVEIT_BEHAVIOUR
  #include "behaviour/moveit/robot_stream.h"
#endif
//---

//---
#if RODYMAN_BEHAVIOUR
  #include "behaviour/RODYMAN/joint.h"
#endif
//---

//---
#if SHERPA_BEHAVIOUR
  #include "behaviour/SHERPA/SHERPA_interface.h"
#endif
//---

//---
#if SAPHARI_BEHAVIOUR
  #include "behaviour/SAPHARI/plan_listener.h"
  #include "behaviour/SAPHARI/go.h"
  #include "behaviour/SAPHARI/give.h"
  #include "behaviour/SAPHARI/take.h"
  #include "behaviour/SAPHARI/receive.h"
  #include "behaviour/SAPHARI/point.h"
  #include "behaviour/SAPHARI/clean.h"
  #include "behaviour/SAPHARI/glue.h"
  #include "behaviour/SAPHARI/place.h"
  #include "behaviour/SAPHARI/attach_bracket.h"
  

  #if SAPHARI_SIMULATION
    #include "behaviour/SAPHARI/simulation_scene.h"
  #else 
    #include "behaviour/SAPHARI/scene.h"
  #endif

#endif

#if SAPHARI_HRI_BEHAVIOUR
  #include "behaviour/SAPHARI/multimodal_hri_framework/fusionEngine.h"
  #include "behaviour/SAPHARI/multimodal_hri_framework/gestureRecognition.h"
//  #include "behaviour/SAPHARI/multimodal_hri_framework/juliusAudioRecognizer.h"
#endif
//---

bool wakeUp(std::string behaviour, std::string instance){
    Behaviour *obj=NULL;

    if(behaviour=="alive")
        return true;
    
    if(behaviour=="forget")
        obj=new ForgetBehaviour(instance);
    else if(behaviour=="listing")
        obj=new ListingBehaviour(instance);
    else if( behaviour=="fg") 
        obj = new FgBehaviour( instance );
    else if(behaviour=="requestStream")
        obj=new RequestStreamBehaviour(instance);       
    else if(behaviour=="inputStream")
        obj=new InputStreamBehaviour(instance);
    else if(behaviour=="rosStream")
        obj=new RosStreamBehaviour(instance);
    else if(behaviour=="$")
        obj=new SequenceBehaviour(instance);

    else if(behaviour=="set")
        obj=new SetBehaviour(instance);
    else if(behaviour=="send")
        obj=new SendBehaviour(instance);

    else if(behaviour=="postGoal")
        obj=new postGoalBehaviour(instance);

  #if SHOW_BEHAVIOUR
    else if(behaviour=="show")
        obj = new ShowBehaviour( instance );
  #endif

  #if TUM_KUKA_BEHAVIOUR
//    else if(behaviour=="kukaTake")
//        obj = new kukaTakeBehaviour( instance );
//    else if(behaviour=="kukaGive")
//        obj = new kukaGiveBehaviour( instance );
//    else if(behaviour=="kukaPlace")
//        obj = new kukaPlaceBehaviour( instance );
//    else if(behaviour=="kukaGripper")
//        obj = new kukaGripperBehaviour( instance );
//    else if(behaviour=="kukaTeach")
//        obj = new kukaTeachBehaviour( instance );
//    else if(behaviour=="kukaDone")
//        obj = new kukaDoneBehaviour( instance );
//    else if(behaviour=="kukaStop")
//        obj = new kukaStopBehaviour( instance );
//    else if(behaviour=="kukaShutdown")
//        obj = new kukaShutdownBehaviour( instance );
    else if(behaviour=="kukaExecute")
        obj = new kukaExecuteBehaviour( instance );
    else if(behaviour=="kukaStream")
        obj = new kukaStreamBehaviour( instance );
    else if(behaviour=="kuka")
        obj = new KukaBehaviour( instance );
    else if(behaviour=="subtask")
        obj = new SubTaskBehaviour( instance );
    else if(behaviour=="repeattask")
        obj = new RepeattaskBehaviour( instance );
    else if(behaviour=="userTake")
        obj = new UserTakeBehaviour( instance );
  #endif

  #if SAPHARI_BEHAVIOUR
    
    //IO
    else if( behaviour == "plan_listener" ) 
      obj = new PlanListenerBehaviour(instance);
  #if SAPHARI_SIMULATION
    else if( behaviour == "simulationscene" ) 
      obj = new simulationSceneBehaviour(instance);
  #else
    else if( behaviour == "scene" )
      obj = new sceneBehaviour( instance );
  #endif

    //Behaviour
    else if( behaviour == "take" ) 
      obj = new takeBehaviour(instance);
    else if( behaviour == "give" )
      obj = new giveBehaviour(instance);
    else if( behaviour == "receive" )
      obj = new receiveBehaviour(instance);
    else if( behaviour == "place" ) 
      obj = new placeBehaviour( instance );
    else if( behaviour == "go" ) 
      obj = new goBehaviour( instance );
    else if( behaviour == "glue" )
      obj = new glueBehaviour(instance);
    else if( behaviour == "pointTo" )
      obj = new pointBehaviour(instance);
    else if( behaviour == "clean" )
      obj = new cleanBehaviour(instance);
    else if( behaviour == "attachbracket" ) 
      obj = new attach_bracketBehaviour( instance );
    
  #endif
  
  #if SAPHARI_HRI_BEHAVIOUR
    else if( behaviour == "fusionEngine" ) 
      obj = new FusionEngineBehaviour(instance);
    else if( behaviour == "gestureRecognition" )
      obj = new GestureRecognitionBehaviour( instance );
//    else if( behaviour == "juliusAudioRecognizer" )
//      obj = new JuliusAudioRecognizerBehaviour( instance );
  #endif

  #if STAGE_BEHAVIOUR
    else if ( behaviour=="gotoxy" )
        obj = new GotoxyBehaviour( instance );
    else if( behaviour=="engineStream")
        obj = new EngineStreamBehaviour( instance );
    else if( behaviour=="laserStream") 
        obj = new LaserStreamBehaviour( instance );
    else if( behaviour=="avoid" )
        obj = new AvoidBehaviour( instance );
  #endif
  
  #if SHERPA_BEHAVIOUR
    else if ( behaviour=="drone" )
        obj = new DroneBehaviour( instance );
    else if( behaviour=="sf")
        obj = new sfBehaviour( instance );
  #endif

  #if MOVEIT_BEHAVIOUR
    else if ( behaviour=="moveit" )
        obj = new MoveitBehaviour( instance );
  #endif
  
  #if RODYMAN_BEHAVIOUR
    else if ( behaviour=="joint" )
        obj = new jointBehaviour( instance );
    else if ( behaviour=="jointStates" )
        obj = new JointStatesBehaviour( instance );
    else if ( behaviour=="moveJointTo" )
        obj = new moveJointToBehaviour( instance );
    else if ( behaviour=="moveJointBy" )
        obj = new moveJointByBehaviour( instance );
  #endif
  
    //execution
    if(obj!=NULL) {
        pthread_t thread_exe;
        pthread_create(&thread_exe, NULL,execution, (void*) obj);
        return true;
    }
    return false;
}
