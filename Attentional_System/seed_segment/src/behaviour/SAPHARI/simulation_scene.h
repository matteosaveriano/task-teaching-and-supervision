#include "../../seed_header.h"
#include "boost/thread.hpp"
#include "TooN/TooN.h"
#include "tf/transform_broadcaster.h"
#include "saphari.h"

#ifndef _SIMULATION_SCENE_B_
#define _SIMULATION_SCENE_B_

typedef struct simu_obj {
  std::string name;
  TooN::Vector<3> pos;

  simu_obj() {
    pos = TooN::Zeros;
  }
}simu_obj;

class simulationSceneBehaviour : public IOBehaviour {

	public:

    void joy_axes_cb( sensor_msgs::Joy axes ) {    
      if( selected_obj != -1 ) {
        TooN::Vector<3> vel = makeVector( axes.axes[0], axes.axes[1], 0.0 );
        scene_obj[selected_obj].pos += vel*0.02*0.8;
      }      
    }

    void joy_but_cb( sensor_msgs::Joy but ) {

      //0 - bracket A
      //1 - human B
      //2 - glue X
      if( but.buttons[0] == 1 ) 
        selected_obj = 0;

      else if( but.buttons[1] == 1 ) 
        selected_obj = 1;
      
      else if( but.buttons[2] == 1 ) 
        selected_obj = 2;
    }


    simulationSceneBehaviour(std::string instance){
      setName(instance2vector(instance)[0]);
      setInstance(instance);
      setRtm(QUIESCENCE);
    
      //joy sub
      joy_axes_sub = nh.subscribe( "/velocity_control", 0, &simulationSceneBehaviour::joy_axes_cb, this);
      joy_but_sub = nh.subscribe( "/joy", 0, &simulationSceneBehaviour::joy_but_cb, this);

      //prendi l'indirizzo della mia istanza
      myAddress=WM->getNodesByInstance(instance)[0];

      robot_pos = TooN::Zeros;
      
      scene_obj[0].name = "bracket_1";
      scene_obj[0].pos = TooN::makeVector(1, 1, 0);

      scene_obj[1].name = "human";
      scene_obj[1].pos = TooN::makeVector(1, 0, 0);

      scene_obj[2].name = "table";
      scene_obj[2].pos = TooN::makeVector(1, -1, 0);


      scene_obj[3].name = "glue";
      scene_obj[3].pos = TooN::makeVector(1, -1, 0);

      selected_obj = -1;

    }
    
   
    bool perceptualSchema(){
      ros::spinOnce();

      this->setRtm( 0.02 );
      
      //publish data on tf
      TooN::Matrix<3> R;
      Fill(R) = 1, 0, 0, 0, 1, 0, 0, 0, 1;
      
      tf_publish( scene_obj[0].pos, R, "world", scene_obj[0].name);
      tf_publish( scene_obj[1].pos, R, "world", scene_obj[1].name);
      tf_publish( scene_obj[2].pos, R, "world", scene_obj[2].name);
      tf_publish( robot_pos, R, "world", "pr2");
      

      //update memory var
      pthread_mutex_lock(&memMutex);
      WMV.set<double> ( "dist(pr2,bracket1)", TooN::norm( robot_pos - scene_obj[0].pos ) );
      WMV.set<double> ( "dist(pr2,human)", TooN::norm( robot_pos - scene_obj[1].pos ) );
      WMV.set<double> ( "dist(pr2,table)", TooN::norm( robot_pos - scene_obj[2].pos ) );             
      WMV.set<double> ( "dist(pr2,glue)", TooN::norm( robot_pos - scene_obj[3].pos ) );                  
   
      pthread_mutex_unlock(&memMutex);
      //

      return true;
      //---
    }

    void motorSchema() {

    }
    void start(){
    }
 
    
	private:

    WM_node* myAddress;
    ros::NodeHandle nh;
    ros::Subscriber joy_axes_sub, joy_but_sub;

    simu_obj scene_obj[4];
    
    TooN::Vector<3> robot_pos;
    int selected_obj;
};

#endif
