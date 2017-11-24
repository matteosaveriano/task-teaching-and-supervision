#ifndef _SCENE_CLASS_
#define _SCENE_CLASS_

#include "../../seed_header.h"
#include "boost/thread.hpp"
#include "TooN/TooN.h"
#include "saphari.h"

//toaster msgs
#include "toaster_msgs/ObjectList.h"
#include "toaster_msgs/RobotList.h"
#include "toaster_msgs/HumanList.h"
#include "toaster_msgs/Entity.h"
#include "toaster_msgs/Object.h"

#define N_OBJECT 21

#define BRACKET_1             0 
#define BRACKET_2             1
#define BRACKET_3             2
#define GLUE_BOTTLE           3       
#define WORK_LOCATION_1       4
#define WORK_LOCATION_2       5
#define WORK_LOCATION_3       6
#define STOCK_TABLE           7
#define ASSEMBLY_SURFACE_1    8
#define ASSEMBLY_SURFACE_2    9
#define ASSEMBLY_SURFACE_3    10
#define PLACEMENT_WL1_1       11
#define PLACEMENT_WL1_2       12
#define PLACEMENT_WL2_1       13
#define PLACEMENT_WL2_2       14
#define PLACEMENT_WL3_1       15
#define PLACEMENT_WL3_2       16
#define PLACEMENT_ST_1        17
#define PLACEMENT_ST_2        18
#define PLACEMENT_ST_3        19
#define PLACEMENT_ST_4        20



typedef struct obj {
  std::string name;
  TooN::Vector<3> pos;

  obj() {
    pos = TooN::Zeros;
  }
}obj;




class sceneBehaviour : public IOBehaviour {

	public:

    void obj_list_cb( toaster_msgs::ObjectList obj_data ) {

      for( int i=0; i<obj_data.objectList.size(); i++ ) {
        if( obj_data.objectList[i].meEntity.name == "BRACKET_1" ) {
          scene_obj[BRACKET_1].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "BRACKET_2" ) {
          scene_obj[BRACKET_2].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "BRACKET_3" ) {
          scene_obj[BRACKET_3].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "GLUE_BOTTLE" ) {
          scene_obj[GLUE_BOTTLE].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "WORK_LOCATION_1" ) {
          scene_obj[WORK_LOCATION_1].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "WORK_LOCATION_2" ) {
          scene_obj[WORK_LOCATION_2].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "WORK_LOCATION_3" ) {
          scene_obj[WORK_LOCATION_3].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "STOCK_TABLE" ) {
          scene_obj[STOCK_TABLE].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "ASSEMBLY_SURFACE_1" ) {
          scene_obj[ASSEMBLY_SURFACE_1].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "ASSEMBLY_SURFACE_2" ) {
          scene_obj[ASSEMBLY_SURFACE_2].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "ASSEMBLY_SURFACE_3" ) {
          scene_obj[ASSEMBLY_SURFACE_3].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "PLACEMENT_WL1_1" ) {
          scene_obj[PLACEMENT_WL1_1].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "PLACEMENT_WL1_2" ) {
          scene_obj[PLACEMENT_WL1_2].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "PLACEMENT_WL2_1" ) {
          scene_obj[PLACEMENT_WL2_1].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "PLACEMENT_WL2_2" ) {
          scene_obj[PLACEMENT_WL2_2].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );

        }
        else if( obj_data.objectList[i].meEntity.name == "PLACEMENT_WL3_1" ) {
          scene_obj[PLACEMENT_WL3_1].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
        else if( obj_data.objectList[i].meEntity.name == "PLACEMENT_WL3_2" ) {
          scene_obj[PLACEMENT_WL3_2].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );

        }
        else if( obj_data.objectList[i].meEntity.name == "PLACEMENT_ST_1" ) {
          scene_obj[PLACEMENT_ST_1].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );

        }
        else if( obj_data.objectList[i].meEntity.name == "PLACEMENT_ST_2" ) {
          scene_obj[PLACEMENT_ST_2].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );

        }
        else if( obj_data.objectList[i].meEntity.name == "PLACEMENT_ST_3" ) {
          scene_obj[PLACEMENT_ST_3].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }           
        else if( obj_data.objectList[i].meEntity.name == "PLACEMENT_ST_4" ) {
          scene_obj[PLACEMENT_ST_4].pos = makeVector( obj_data.objectList[i].meEntity.positionX, obj_data.objectList[i].meEntity.positionY, 0.0 );
        }
      }
      first_obj_msg = true;
    }

    void rob_list_cb( toaster_msgs::RobotList rob_data ) {
      robot_pos = makeVector( rob_data.robotList[0].meAgent.meEntity.positionX,
                              rob_data.robotList[0].meAgent.meEntity.positionY,
                              0.0);

                                     // std::cout << "robot_pos: " << robot_pos << std::endl;
  
    }
    void hum_list_cb( toaster_msgs::HumanList hum_data ) {


       human_pos = makeVector( hum_data.humanList[0].meAgent.meEntity.positionX,
                              hum_data.humanList[0].meAgent.meEntity.positionY,
                              0.0);  

    }

    sceneBehaviour(std::string instance){
      setName(instance2vector(instance)[0]);
      setInstance(instance);
      setRtm(QUIESCENCE);
    
      first_obj_msg = false;
      obj_list_sub = nh.subscribe("/pdg/objectList", 0, &sceneBehaviour::obj_list_cb, this);
      rob_list_sub = nh.subscribe("/pdg/robotList", 0, &sceneBehaviour::rob_list_cb, this);
      hum_list_sub = nh.subscribe("/pdg/humanList", 0, &sceneBehaviour::hum_list_cb, this);

         //prendi l'indirizzo della mia istanza
      myAddress=WM->getNodesByInstance(instance)[0];

      robot_pos = TooN::Zeros;

      scene_obj[BRACKET_1].name = LAAS2seed_param("BRACKET_1");
      scene_obj[BRACKET_2].name = LAAS2seed_param("BRACKET_2") ;           
      scene_obj[BRACKET_3].name = LAAS2seed_param("BRACKET_3") ;           
      scene_obj[GLUE_BOTTLE].name = LAAS2seed_param("GLUE_BOTTLE");       
      scene_obj[WORK_LOCATION_1].name = LAAS2seed_param("WORK_LOCATION_1");   
      scene_obj[WORK_LOCATION_2].name = LAAS2seed_param("WORK_LOCATION_2");  
      scene_obj[WORK_LOCATION_3].name = LAAS2seed_param("WORK_LOCATION_3");   
      scene_obj[STOCK_TABLE].name = LAAS2seed_param("STOCK_TABLE");    
      scene_obj[ASSEMBLY_SURFACE_1].name = LAAS2seed_param("ASSEMBLY_SURFACE_1");  
      scene_obj[ASSEMBLY_SURFACE_2].name = LAAS2seed_param("ASSEMBLY_SURFACE_2");
      scene_obj[ASSEMBLY_SURFACE_3].name = LAAS2seed_param("ASSEMBLY_SURFACE_3") ;   
      scene_obj[PLACEMENT_WL1_1].name = LAAS2seed_param("PLACEMENT_WL1_1");     
      scene_obj[PLACEMENT_WL1_2].name = LAAS2seed_param("PLACEMENT_WL1_2");     
      scene_obj[PLACEMENT_WL2_1].name = LAAS2seed_param("PLACEMENT_WL2_1");     
      scene_obj[PLACEMENT_WL2_2].name = LAAS2seed_param("PLACEMENT_WL2_2");  
      scene_obj[PLACEMENT_WL3_1].name = LAAS2seed_param("PLACEMENT_WL3_1");    
      scene_obj[PLACEMENT_WL3_2].name = LAAS2seed_param("PLACEMENT_WL3_2");     
      scene_obj[PLACEMENT_ST_1].name = LAAS2seed_param("PLACEMENT_ST_1");        
      scene_obj[PLACEMENT_ST_2].name = LAAS2seed_param("PLACEMENT_ST_2");        
      scene_obj[PLACEMENT_ST_3].name = LAAS2seed_param("PLACEMENT_ST_3");       
      scene_obj[PLACEMENT_ST_4].name = LAAS2seed_param("PLACEMENT_ST_4");       
      robot_pos = Zeros;
    }
    
   
    bool perceptualSchema(){
      ros::spinOnce();

      this->setRtm( 0.02 );
      
      //publish data on tf
      TooN::Matrix<3> R;
      Fill(R) = 1, 0, 0, 0, 1, 0, 0, 0, 1;
      
      // tf_publish( scene_obj[0].pos, R, "world", scene_obj[0].name);
      // tf_publish( scene_obj[1].pos, R, "world", scene_obj[1].name);
      // tf_publish( scene_obj[2].pos, R, "world", scene_obj[2].name);
      // tf_publish( robot_pos, R, "world", "pr2");
      

      //update memory var
      pthread_mutex_lock(&memMutex);

      if( first_obj_msg ) {
        for(int i=0; i<N_OBJECT;i++) {
          std::stringstream ss;
          ss << "dist(pr2," << scene_obj[i].name << ")";
          //std::cout << "Obj[" << i << "]: " << ss.str() << " " << TooN::norm( robot_pos - scene_obj[i].pos ) << std::endl;
          WMV.set<double> ( ss.str(), TooN::norm( robot_pos - scene_obj[0].pos ) );
        }
      }

      //TODO:: HUMAN UPDATE
      std::stringstream ss;
      ss << "dist(pr2,human)";
      WMV.set<double> ( ss.str(), TooN::norm( robot_pos - human_pos ) );


// std::cout << "Work_location_1: " << scene_obj[WORK_LOCATION_1].pos << " Dist: " <<  TooN::norm( robot_pos - scene_obj[WORK_LOCATION_1].pos ) << std::endl;
// std::cout << "BRACKET_1: " << scene_obj[BRACKET_1].pos << " Dist: " <<  TooN::norm( robot_pos - scene_obj[BRACKET_1].pos ) << std::endl;
// std::cout << "human: " << human_pos << " Dist: " <<  TooN::norm( robot_pos - human_pos )  << std::endl;

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
    obj scene_obj[N_OBJECT];
    bool first_obj_msg;

    TooN::Vector<3> robot_pos;
    TooN::Vector<3> human_pos;

    ros::Subscriber obj_list_sub;
    ros::Subscriber rob_list_sub;
    ros::Subscriber hum_list_sub;
};

#endif
