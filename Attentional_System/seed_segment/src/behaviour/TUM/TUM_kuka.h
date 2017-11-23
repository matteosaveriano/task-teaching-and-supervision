#include "../../seed_header.h"

#define KUKA_OJB_MAX_DISTANCE 2

class RepeattaskBehaviour : public Behaviour{
    public:
    RepeattaskBehaviour(std::string instance){
      setName(instance2vector(instance)[0]);
      setInstance(instance);
      setRtm(QUIESCENCE);
    }

    bool perceptualSchema(){

        return true;
    }
    void motorSchema(){
        pthread_mutex_lock(&memMutex);

        WMV.set<double>("water.used", 0 );
        WMV.set<double>("coffee.used", 0 );
        WMV.set<double>("cup.used", 0 );
        WMV.set<double>("spoon.used", 0 );
        WMV.set<double>("tea.used", 0 );
        WMV.set<double>("lever.used", 0 );
        WMV.set<double>("button.used", 0 );

        remove(WM->getNodesByInstance(this->getInstance())[0]);
        pthread_mutex_unlock(&memMutex);
    }
    void exit() {
    }
    void start() {
    }
protected:
};

class UserTakeBehaviour : public Behaviour{
    public:
    UserTakeBehaviour(std::string instance){
      setName(instance2vector(instance)[0]);
      setInstance(instance);
      setRtm(QUIESCENCE);

      object = instance2vector(instance)[1];
    }

    bool perceptualSchema(){

        return true;
    }
    void motorSchema(){
        pthread_mutex_lock(&memMutex);

        WMV.set<double>(object + ".used", 1 );
//        WMV.set<double>("coffee.used", 0 );
//        WMV.set<double>("spoon.used", 0 );
//        WMV.set<double>("tea.used", 0 );

        remove(WM->getNodesByInstance(this->getInstance())[0]);
        pthread_mutex_unlock(&memMutex);
    }
    void exit() {
    }
    void start() {
    }
protected:
    std::string object;
};

//KUKA GENERIC ACTION EXECUTION <under development>
/*  kuka(action,envFeature) | kuka(action)
*       action: command to be send to the kuka robot.
*           [list] "gripper(close)" "gripper(open)" "shutdown" "teach" "done" "stop"
*       envFeature: object or feature of the environment for the bottom-up stimulus.
*                   Could be NOT specified if for a fixed stimulus.
*           [eg] "water" "coffee" "milk"
*/
class KukaBehaviour : public Behaviour{
    public:
    KukaBehaviour(std::string instance){
      setName(instance2vector(instance)[0]);
      setInstance(instance);
      setRtm(QUIESCENCE);
      action = instance2vector(instance)[1];

      if(instance2vector(instance).size()>2)
          feature = instance2vector(instance)[2];
      else
          feature = "null";
    }

    bool perceptualSchema(){

        if(feature=="null") {
            pthread_mutex_lock(&memMutex);
            fdistance = WMV.get<double>(feature + ".distance");
            pthread_mutex_unlock(&memMutex);
        }

        return true;
    }
    void motorSchema(){
        pthread_mutex_lock(&memMutex);
        if(feature=="null")
            updateRtm(0,KUKA_OJB_MAX_DISTANCE,0); //setRtm(0.01);
        else //if (WMV.get<double>("kuka.teach")!=1 )
            updateRtm(fdistance,KUKA_OJB_MAX_DISTANCE,0);

        //if(WMV.get<double>(this->getInstance() + "known"))
        send< std::string >("kukaStream","kukaStream.action",action );

        //remove(WM->getNodesByInstance(this->getInstance())[0]);
        pthread_mutex_unlock(&memMutex);
    }
    void exit() {
    }
    void start() {
    }
protected:
    std::string action;
    std::string feature;
    double fdistance;
};


//GENERIC HIGH LEVEL ACTION REPRESENTATION
/*  subtask(action,target)
*       action: high level name of action.
*           [eg] "take" "give" "place" "pour"
*       target: object/position/etc. in the environment for the bottom-up stimulus.
*           [eg] "water" "coffee" "milk"
*/
class SubTaskBehaviour : public Behaviour{
    public:
    SubTaskBehaviour(std::string instance){
      setName(instance2vector(instance)[0]);
      setInstance(instance);
      setRtm(QUIESCENCE);
      action = instance2vector(instance)[1];
      target = instance2vector(instance)[2];
    }

    bool perceptualSchema(){

        pthread_mutex_lock(&memMutex);
        fdistance = WMV.get<double>(target + ".distance");
        pthread_mutex_unlock(&memMutex);

        return true;
    }
    void motorSchema(){
        pthread_mutex_lock(&memMutex);

        updateRtm(fdistance,KUKA_OJB_MAX_DISTANCE,0);

        pthread_mutex_unlock(&memMutex);
    }
    void exit() {
    }
    void start() {
    }
protected:
    std::string action;
    std::string target;
    double fdistance;
};


//KUKA GENERIC TRAJECTORY EXECUTION
/*  kukaExecute(trajectory_id)
*       trajectory_id: id of the trajectory for the kuka robot.
*           [eg] N/A
*/
class kukaExecuteBehaviour : public Behaviour{
    public:
    kukaExecuteBehaviour(std::string instance){
      setName(instance2vector(instance)[0]);
      setInstance(instance);
      setRtm(QUIESCENCE);
      trajectory_id = instance2vector(instance)[1];
      feature = instance2vector(instance)[2];

    }

    bool perceptualSchema(){

        if(feature != "world") {
            pthread_mutex_lock(&memMutex);
            fdistance = WMV.get<double>(feature + ".distance");
            pthread_mutex_unlock(&memMutex);
        }

        return true;
    }
    void motorSchema(){
        pthread_mutex_lock(&memMutex);
        if(feature == "world")
            updateRtm(0,KUKA_OJB_MAX_DISTANCE,0);//setRtm(0.01);
        else
            updateRtm(fdistance,KUKA_OJB_MAX_DISTANCE,0);

        if( WMV.get<double>("kuka.teach") != 1 )
            send< std::string >("kukaStream","kukaStream.action", "execute(" + trajectory_id + "," + feature + ")" );

        //remove(WM->getNodesByInstance(this->getInstance())[0]);
        pthread_mutex_unlock(&memMutex);
    }
    void exit() {
    }
    void start() {
    }
protected:
    std::string trajectory_id;
    std::string feature;
    double fdistance;
};


/* **************** KUKASTREAM **************** */
std::string kuka_dist = "(tea,10.0,coffee,10.0,lever,10.0,button,10.0,cup,1.0,water,10.0,milk,10.0,spoon,10.0)";
std::string kuka_return;

void ros_kukaDist_callback(const std_msgs::String::ConstPtr& msg){
    kuka_dist = msg->data;
}

void ros_kukaReturn_callback(const std_msgs::String::ConstPtr& msg){
    kuka_return = msg->data;
}

class kukaStreamBehaviour : public Behaviour{
    public:
    kukaStreamBehaviour(std::string instance){
      setName(instance2vector(instance)[0]);
      setInstance(instance);
      setRtm(QUIESCENCE);

      kuka_pub = nh.advertise<std_msgs::String>("kuka_action", 1000);

      kuka_dist_sub=nh.subscribe("object_dist", 2, ros_kukaDist_callback);
      kuka_return_sub=nh.subscribe("kuka_return", 2, ros_kukaReturn_callback);

      last_segment = "";
      last_kuka_action = "";
      updating_instance = "";

      resetObjectMap();

      WMV.set<double>("kukahand.free",1);
      WMV.set<std::string>("kukahand.object", "null" );
      WMV.set<double>("kuka.teach",0);
    }

    bool perceptualSchema(){

        ros::spinOnce();

        if(kuka_dist == "")
            return true;

        pthread_mutex_lock(&memMutex);
        //setRtm(0.1);
        //get distances from rostopic in form: (obj1,dist1,obj2,dist2 ...)
        resetObjectMap();
        dist_vector = instance2vector(kuka_dist);
        for(size_t i=1; i<dist_vector.size(); i=i+2){
          //WMV.set<double>(dist_vector[i] + ".distance", atof(dist_vector[i+1].c_str()) );
            obj_map[dist_vector[i]] = atof(dist_vector[i+1].c_str());
          //std::cout<<"dist: "<<dist_vector[i]<< " "<<atof(dist_vector[i+1].c_str())<<"\n";
        }
        kuka_dist = "";

        WMV.set<double>("water.distance", obj_map["water"]);
        WMV.set<double>("milk.distance", obj_map["milk"]);
        //WMV.set<double>("lever.distance", obj_map["lever"]);
        WMV.set<double>("tea.distance", obj_map["tea"]);
        WMV.set<double>("spoon.distance", obj_map["spoon"]);
        //WMV.set<double>("button.distance", obj_map["button"]);
        WMV.set<double>("coffee.distance", obj_map["coffee"]);
        WMV.set<double>("cup.distance", obj_map["cup"]);
        
       // Pezzottone!!!!!!!! //
        if((WMV.get<double>("coffee.used")==1)){
            WMV.set<double>("coffee.distance", 10.0);
            WMV.set<double>("lever.distance", obj_map["lever"]);
        }
        else{
            WMV.set<double>("lever.distance", 10.0);
        }
        if((WMV.get<double>("lever.used")==1)){
            WMV.set<double>("lever.distance", 10.0);  
            WMV.set<double>("button.distance", obj_map["button"]);
        }
        else{
            WMV.set<double>("button.distance", 10.0);
        }
        
        if((WMV.get<double>("cup.used")==1))
            WMV.set<double>("cup.distance", 10.0);   

        pthread_mutex_unlock(&memMutex);
        return true;
}
    void motorSchema(){
        pthread_mutex_lock(&memMutex);

        std::string kukaAction = WMV.get<std::string>("kukaStream.action");
        double competitorsNumber = WMV.get<double>("kukaStream.action.count");

        if (competitorsNumber == 1){
            //send kukaAction via topic
            kuka_msg.data = kukaAction;
            WMV.set<double>("kukaStream.action.count",0);
        }
        else{
            //send nullAction via topic
            //std::cout<<"action: "<<kukaAction<<" count: "<<competitorsNumber<<"\n";
            kuka_msg.data = "null";
            kukaAction = "null";
            WMV.set<double>("kukaStream.action.count",0);
        }

        //publish only if have a new action
        //if(kukaAction != last_kuka_action)
            kuka_pub.publish(kuka_msg);

        last_kuka_action = kukaAction;

        if(kuka_return != ""){
            return2wmv(kuka_return);
            kuka_return = "";
        }

        pthread_mutex_unlock(&memMutex);
    }
    void exit() {
    }
    void start() {
    }
    void return2wmv(std::string kr){
        std::vector< std::string > res = instance2vector(kr);
        if(res[0] == "shutdown" ){
            //forget alive
            remove(WM->getNodesByInstance("alive")[0]);
        }
        else if(res[0] == "gripper" && res[1] == "open" ){

            //learn the most emphasized action if in teach-mode
            if(WMV.get<double>("kuka.teach")==1)
                learnBehavior("kuka(gripper(open))");//,last_segment + ".done");
            //else
                //WMV.set< double >("human." + WMV.get<std::string>("kukahand.object"),1);

            WMV.set< double >(WMV.get<std::string>("kukahand.object") + ".used",1);

            WMV.set< double >( WMV.get<std::string>("kukahand.object") + ".taken", 0 );
            WMV.set< double >("kukahand.free",1);
            WMV.set<std::string>("kukahand.object", "null" );

//            if(WM->getNodesByInstance("kukaGripper(open)").size()!=0)
//                remove(WM->getNodesByInstance("kukaGripper(open)")[0]);
        }
        else if(res[0] == "gripper" && res[1] == "close" ){

            //learn the most emphasized action if in teach-mode
            if(WMV.get<double>("kuka.teach")==1)
                learnBehavior("kuka(gripper(close))");//,last_segment + ".done");

            WMV.set< double >(res[2] + ".taken", 1);
            WMV.set< double >("kukahand.free",0);
            WMV.set<std::string>("kukahand.object", res[2] );

            //WMV.set< double >(res[1] + ".taken", 1);
            //WMV.set< double >("kukahand.free",0);

//            if(WM->getNodesByInstance("kukaGripper(close)").size()!=0)
//                remove(WM->getNodesByInstance("kukaGripper(close)")[0]);
        }
        else if(res[0] == "done" ){
            WMV.set<double>("kuka.teach", 0 );

            WMV.set<double>("water.used", 0 );
            WMV.set<double>("coffee.used", 0 );
            WMV.set<double>("spoon.used", 0 );
            WMV.set<double>("tea.used", 0 );
            WMV.set<double>("button.used", 0 );
            WMV.set<double>("lever.used", 0 );
            WMV.set<double>("cup.used", 0 );
            
            //reset all amplifications given by the teaching phase
            if(WM->getNodesByInstance("preparecoffee").size()>0)
                restore_amplification(WM->getNodesByInstance("preparecoffee")[0]);

            if(WM->getNodesByInstance("preparetea").size()>0)
                restore_amplification(WM->getNodesByInstance("preparetea")[0]);

//            std::vector<WM_node *> task_nodes = WM->getNodesByName("subtask");
//            for(int i=0; i<task_nodes.size(); i++)
//                task_nodes[i]->amplification = 0;

            remove(WM->getNodesByInstance("kuka(done)")[0]);
        }
        else if(res[0] == "teach" ){
            WMV.set<double>("kuka.teach", 1 );
        }
        else if(res[0] == "segment"){

            std::string new_instance,new_goal;

            //save the segment instance with the ID and the object
            new_instance = "kukaExecute(" + res[1] + "," + res[4] + ")";

            //check if new segment has a releaser
//            if(res[2] == "true" && last_segment != "")
//                new_releaser = last_segment + ".done";
//            else
//                new_releaser = "\"TRUE\"";

            //check if new segment has a goal
            if(res[3] == "true")
                new_goal = new_instance + ".done";
            else
                new_goal = "null";

            learnBehavior(new_instance,new_goal);

            last_segment = new_instance;
        }
        else if(res[0] == "execute" ){
            std::string executed = "kukaExecute(" + res[1] + "," + res[2] + ")";
            WMV.set<double>(executed + ".done", 1 );
            std::cout<<"KUKA, DONE: " << executed << "\n";
        }
    }
    void learnBehavior(std::string new_instance ,
                       std::string new_goal = "null" ){
        std::string new_releaser;
        std::stringstream ss;

        //sort all nodes of WM
        std::vector<WM_node *> sortedList = sortNodes(WM->tree2list());
        //for each node of the sorted list
        for(size_t i=0;i<sortedList.size();i++){
            //if that node can be updated
//            if(sortedList[i]->name == "kukaTake" ||
//               sortedList[i]->name == "kukaGive" ||
//               sortedList[i]->name == "kukaPlace" ){
            if(sortedList[i]->name == "subtask") {

                //insert the releaser while updating the same instance
                if(sortedList[i]->instance == updating_instance && last_segment != "")
                    new_releaser = last_segment + ".done";
                //else a new instance is started
                else{
                    std::cout<<"KUKA: NEW ACTION STARTS/n";
                    new_releaser = "\"TRUE\"";
                    updating_instance = sortedList[i]->instance;
                }

                //build the PROLOG command
                ss << "updateSchema(" << sortedList[i]->instance << ","
                                      << new_instance << ","
                                      << new_releaser << ","
                                      << new_goal << ")";

                std::cout<<"toPROLOG: "<<ss.str() << "\n";

                //send the command to PROLOG
                post_goal(ss.str().c_str());
                EC_resume();

                //send the command to PROLOG
                post_goal(std::string("saveLTM").c_str());
                EC_resume();
                std::cout<<"LTM saved\n";

                //update the node in WM according with the new schema
//                WM_node *new_node = sortedList[i]->addSon(new_instance);
//                if(new_releaser == "\"TRUE\"")
//                    new_node->releaser.push_back("TRUE");
//                else
//                    new_node->releaser.push_back(new_releaser);

                std::vector<WM_node *> update_nodes = WM->getNodesByInstance(sortedList[i]->instance);
                for(size_t j=0; j<update_nodes.size(); j++){
                    WM_node *new_node = update_nodes[j]->addSon(new_instance);
                    if(new_releaser == "\"TRUE\"")
                        new_node->releaser.push_back("TRUE");
                    else
                        new_node->releaser.push_back(new_releaser);
                }
                std::cout<<"KUKA: "<< sortedList[i]->instance <<" UPDATED\n";
                return;
            }
        }
    }
    void restore_amplification(WM_node * root){
        root->amplification = 0;
        for(size_t i=0; i<root->son.size(); i++)
            restore_amplification(root->son[i]);
    }

    void resetObjectMap(){
        obj_map["water"] = 10;
        obj_map["milk"] = 10;
	obj_map["lever"] = 10;
	obj_map["button"] = 10;
        obj_map["tea"] = 10;
        obj_map["coffee"] = 10;
        obj_map["spoon"] = 10;
	obj_map["cup"] = 10;
    }

protected:
    ros::NodeHandle nh;
    ros::Publisher kuka_pub;
    ros::Subscriber kuka_dist_sub,kuka_return_sub;
    std_msgs::String kuka_msg;
    std::vector < std::string > dist_vector;
    std::string last_segment;
    std::string last_kuka_action;
    std::string updating_instance;
    std::map<std::string, double> obj_map;
};
