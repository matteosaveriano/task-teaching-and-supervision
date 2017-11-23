#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include "ros/package.h"

#define SIZE 2
#define MAXLINE 4096
#define LENGTH 8

using namespace std;


bool esci = false;

void sig_handler(int signo){

    if (signo == SIGINT){
        system("rm -R recFiles");
		esci = true;
	}
}


int main(int argc, char** argv){
	
	ros::init(argc,argv,"speech_rec");
	
    system("mkdir -p recFiles");

	char buf[MAXLINE];
	char time_len[LENGTH];
	
	ros::NodeHandle nodeH;
	ros::Publisher speechPub = nodeH.advertise<std_msgs::String>("seed_stream",0);
	std_msgs::String msg;
	
    if (signal(SIGINT, sig_handler) == SIG_ERR)
		cout << "can't catch signal" << endl;
			
	
	FILE *pf;
	char *s;
	char *w;
	string lineFile,line;
	string recognized;
	stringstream speechStream;

    string NODE_HOME_PATH = ros::package::getPath("speech_seed");

    string SYS_HOME_PATH = getenv("HOME");

    string command("padsp julius-4.3.1 -input mic -demo -C " + NODE_HOME_PATH + "/asr/munich/julian.jconf"); // "/asr/airbus/julian.jconf"); // \/home\/prisma-airobots\/catkin_ws\/src\/speech_seed\/asr\/julian.jconf");
    pf = popen(command.c_str(),"r");
	
	while (ros::ok() && !esci){
		
		if (pf != NULL){
			while (fgets (buf , MAXLINE , pf) != NULL){
				
				w = strstr(buf, "write file");
				if (w != NULL ){
					lineFile = buf;
					int start = lineFile.find("/");
					lineFile = lineFile.substr(start);
					
				}
				
				s = strstr(buf,"sentence1: <s> ");
				if (s != NULL){
					cout << lineFile << endl;
					puts(buf);
					
					//~ //start reading length of speech file in seconds
					//~ FILE *ptime;
					//~ stringstream cmdSs("");
					//~ cmdSs << "ecalength -s " << lineFile;
					//~ ptime = popen(cmdSs.str().c_str(),"r");
					//~ fgets(time_len,LENGTH,ptime);
					//~ puts(time_len);
					//~ pclose(ptime);
					//~ //end reading length of speech file
					
					line = buf;
					int start = line.find(">");
					int end = line.find("</", start);
					recognized = line.substr(start+2,end-start-3);
				
					std::transform(recognized.begin(), recognized.end(), recognized.begin(), ::tolower);
					//std::replace( recognized.begin(), recognized.end(), ' ', '.');
					///unina
					//~ size_t recfind=recognized.find("computer",0);
					///tolosa
					size_t recfind=recognized.find("robots",0);
					if(recfind!=string::npos){
						//~ std::cout << "speech: " << recognized << "\n";
						//pubblicare il messaggio
						speechStream.str("");
						speechStream << "en(\"" << recognized << "\")" ;
						msg.data = speechStream.str();
						speechPub.publish(msg);
					}else 
						std::cout<<"speech rejected\n";
					
				}
			}
				
		}
	}
	pclose(pf);
	exit(0);
}
