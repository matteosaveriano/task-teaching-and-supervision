#include "LWR_seed_control/robotBehaviourManager.h"
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
        commLine << "addr2line %p -e " << ros::package::getPath("LWR_seed_control")+"/bin/learnTask";
        sprintf(syscom, commLine.str().c_str() , trace[i]); //last parameter is the name of this app
        //std::cout<<syscom<<"\n";
        system(syscom);
    }

    exit(1);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "learnTask");
    ros::NodeHandle node;


    signal(SIGSEGV, faultHandler);

    //std::string objectFile = "/data/objects_database_addWater.xml";
    std::string objectFile = "/data/objects_database_coffee.xml";
    //std::string objectFile = "/data/objects_database_tea.xml";

    robotBehaviourManager *behMan = new robotBehaviourManager(node, objectFile);

    ros::spin();

    // Save learned task
    std::cout << "Save learned task before exit? [Y/n]" << std::endl;
    std::string c;
    std::cin >> c;
    if(c=="y" || c=="Y"){
        std::cout << "Saving data" << std::endl;
        std::string packPath = ros::package::getPath("LWR_seed_control");
        //behMan->saveLearnedTask(packPath+"/data/addWater/");
        behMan->saveLearnedTask(packPath+"/data/prepareCoffee/");
        //behMan->saveLearnedTask(packPath+"/data/prepareTea/");

        packPath = ros::package::getPath("kuka_seed_commands");
        //behMan->saveObjectList(packPath+"/data/objects_database_addWater_new.xml");
        behMan->saveObjectList(packPath+"/data/objects_database_coffee_new.xml");
        //behMan->saveObjectList(packPath+"/data/objects_database_tea_new.xml");
    }

    delete behMan;

    return(0);
}
