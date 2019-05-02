#include "lwr_seed_control/robotBehaviourManager.h"
#include "std_msgs/String.h"

#include <signal.h>
#include <execinfo.h>

void faultHandler(int sig){
    void *trace[16];
    size_t size;
    char **messages = (char **)NULL;

    // get void*'s for all entries on the stack
    size = backtrace(trace, 16);

    // print out all the frames to stderr
    if(sig==11)
        fprintf(stderr, "SEED-ERROR (Segmentation Fault) backtrace:\n");
    else
        fprintf(stderr, "SEED-ERROR (signal %d) backtrace:\n", sig);

    //standard backtrace (no error line)
    //backtrace_symbols_fd(trace, size, STDERR_FILENO);

    //backtrace
    messages = backtrace_symbols(trace, size);
    for (int i = 1; i < size; ++i) {
        printf("[bt] #%d %s\n", i, messages[i]);
        char syscom[256];
        std::stringstream commLine;
        commLine << "addr2line %p -e " << "/home/hwadong/cotesys-Matteo/multimodal_interaction_and_teaching/lwr_seed_control/bin/executeTask";
        sprintf(syscom, commLine.str().c_str() , trace[i]); //last parameter is the name of this app
        //std::cout<<syscom<<"\n";
        system(syscom);
    }

    exit(1);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "executeTask");
    ros::NodeHandle node;


    signal(SIGSEGV, faultHandler);

    //std::string objectFile = "/data/objects_database_addWater_new.xml";
    std::string objectFile = "/data/objects_database_coffee_new.xml";
    //std::string objectFile = "/data/objects_database_tea_new.xml";

    robotBehaviourManager *behMan = new robotBehaviourManager(node, objectFile);

    // Load task
    std::string packPath = ros::package::getPath("lwr_seed_control");
    //std::string task = "/data/addWater/";
    std::string task = "/data/prepareCoffee/";
    //std::string task = "/data/prepareTea/";
    if(behMan->loadLearnedTask(packPath+task)){
        ros::spin();
    }

    delete behMan;

    return(0);
}
