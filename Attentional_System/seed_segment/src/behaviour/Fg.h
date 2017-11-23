/* 
 * File:   mie_fg.h
 * Author: hargalaten
 *
 * Created on 14 gennaio 2013, 18.32
 */

#ifndef MIE_FG_H
#define	MIE_FG_H

#include "../seed_header.h"
#include <fstream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <time.h>
//#include "../graphLib/GraphUtils.h"

#define HISTORYTIME 60

#define FIGURE_WIDTH 660
#define FIGURE_HEIGHT 1000

#define FG_WIDTH 600
#define FG_HEIGHT 200
#define FG_BASE 80
#define FG_START 20
//spazio tra il grafico ed i testi
#define FG_SPACING 10

struct WM_graph{
    std::string schema;
    std::string subSchema;
    cv::Scalar color;
    float value [HISTORYTIME];
    double magnitude;
} ;

class FgBehaviour : public Behaviour{
public:
    FgBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        
        fgPath="/home/hargalaten/Scrivania/ros_sources/roseed/src/frequencyGraph.txt";
        
        //system("gnome-terminal --command=\"tail -f /home/hargalaten/Scrivania/ros_sources/roseed/src/frequencyGraph.txt\" &");
        
        cv::RNG rng( 0xFFFFFFFF );
        
        std::string clear=instance;
        
        //pulisci l'istanza dai caratteri di inizio e fine lista
        clear.erase(std::remove(clear.begin(), clear.end(), '['), clear.end());
        clear.erase(std::remove(clear.begin(), clear.end(), ']'), clear.end());
        
        
        schemaNumber=instance2vector(clear).size()-1;
        
        //set static colors
        std::vector< cv::Scalar > staticColor;
        staticColor.push_back( cv::Scalar(0,255,0) );
        staticColor.push_back( cv::Scalar(0,0,255) );
        staticColor.push_back( cv::Scalar(0,215,255) );
        staticColor.push_back( cv::Scalar(0,128,255) );
        staticColor.push_back( cv::Scalar(255,0,0) );
        
        if(clear=="fg()")
            schemaNumber=0;
        
        for(int i=0;i<schemaNumber;i++){
            rng.state=rand() % 10000 + 1;
            struct WM_graph app;
            app.schema = instance2vector(clear)[i+1];
            app.subSchema = "";
            //app.color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 200));
            app.color=staticColor[i];
            init(app.value);
            app.magnitude=0;
            history.push_back(app);
        }
        
        cv::namedWindow("FrequencyGraph", cv::WINDOW_NORMAL );
    
    }
    bool perceptualSchema(){
        return true;
    }
    void motorSchema(){
        
        std::ofstream file;
        double apprtm;
        file.open(fgPath.c_str(),std::ios::app);
        pthread_mutex_lock(&memMutex);
        
        setRtm(0.5);
        
        if(dead()) {
            file.close();
            return;
        }
        
        for(int i=0;i<schemaNumber;i++){
            if(WM->getNodesByInstance(history[i].schema).size()!=0){
                
                history[i].subSchema="";
                
                if(WM->getNodesByInstance(history[i].schema)[0]->abstract){
                    
                    std::vector< WM_node* > bestList;
                    for(int j=0; j<WM->getNodesByInstance(history[i].schema).size(); j++){
                        bestList.push_back(sortNodes(WM->getNodesByInstance(history[i].schema)[0]->tree2list())[0]);
                    }
                    
                    history[i].subSchema=sortNodes(bestList)[0]->instance;
                    
                    WM_node * pivot=getReleasedInstance(history[i].subSchema);
                    
                    apprtm=pivot->rtm;
                    history[i].magnitude=WM->getInstanceMagnitude(history[i].subSchema);
                }
                else{
                    
                    WM_node * pivot=getReleasedInstance(history[i].schema);
                    
                    apprtm=pivot->rtm;
                    history[i].magnitude=WM->getInstanceMagnitude(history[i].schema);
                }
                
                
                if(apprtm == QUIESCENCE)
                    apprtm=1.1;
                
                shift(history[i].value,apprtm);
                
            }
            else{
                shift(history[i].value,1);
                history[i].magnitude=0;
            }
        }
        
        listAll(WM,&file,"");
        
        pthread_mutex_unlock(&memMutex);
        
        //scala l'altezza in base agli schemi da visualizzare (MASSIMO 5)
        int scaled_height= 140 + (110 * schemaNumber );
        //crea la matrice con l'altezza scalata
        cv::Mat image( scaled_height, FIGURE_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
        int floor=FG_HEIGHT-FG_BASE;
        //disegna il grafico complessivo di tutti gli schemi
        for (int i = 0; i < schemaNumber; i++) {
            for (int j=1; j < HISTORYTIME; j++) {
                cv::line(image,cv::Point(FG_START+((j-1)*10),floor-( (1-history[i].value[j-1])*100)),
                        cv::Point(FG_START+(j*10),floor-( (1-history[i].value[j])*100)), history[i].color,2,8,0 );
            }
        }
        //crea le linee orizzontali del grafico
        std::stringstream ss;
        for(int i=1;i<10;i++){
            cv::line(image,cv::Point(FG_START,floor-(i*10)),cv::Point(FG_START+FG_WIDTH,floor-(i*10)), cv::Scalar(190,190,190),1,8,0);
            ss<<i*10;
            cv::putText(image,ss.str(),cv::Point(FG_START+FG_WIDTH+1,floor-(i*10)),cv::FONT_HERSHEY_SIMPLEX,0.3,cv::Scalar(0,0,0),1,8,false);
            ss.str("");
        }
        //crea la linea orizzontale di altezza 100
        cv::line(image, cv::Point(FG_START, floor - 100), cv::Point(FG_START + FG_WIDTH, floor - 100), cv::Scalar(0, 0, 0), 1, 8, 0);
        ss << 100;
        cv::putText(image, ss.str(), cv::Point(FG_START + FG_WIDTH + 1, floor - 100), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, false);
        ss.str("");
        //crea la linea orizzontale di altezza 0
        cv::line(image, cv::Point(FG_START, floor), cv::Point(FG_START + FG_WIDTH, floor), cv::Scalar(0, 0, 0), 1, 8, 0);
        ss << 0;
        cv::putText(image, ss.str(), cv::Point(FG_START + FG_WIDTH + 1, floor), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, false);
        ss.str("");
        //crea la linea orizzontale di altezza -1 (quiescenza)
        cv::line(image,cv::Point(FG_START,floor+10),cv::Point(FG_START+FG_WIDTH,floor+10), cv::Scalar(0,0,0),1,8,0);
        cv::putText(image,"(q)",cv::Point(FG_START+FG_WIDTH+1,floor+10),cv::FONT_HERSHEY_SIMPLEX,0.3,cv::Scalar(0,0,0),1,8,false);
        
        cv::rectangle(image,cv::Point(FG_START,5),cv::Point(FIGURE_WIDTH-10,floor+15), cv::Scalar(0,0,0),2,8,0);
        
        int c=0;//l=0;
        for(int i=0;i<schemaNumber;i++){
//            if(c==5){
//                l++;
//                //c++;
//                c=0;
//            }
            if(history[i].subSchema=="")
                ss<<history[i].schema<<" "<<history[i].magnitude<<"|"<<history[i].value[HISTORYTIME-1];
            else
                ss<<history[i].schema<<"/"<<history[i].subSchema<<" "<<history[i].magnitude<<"|"<<history[i].value[HISTORYTIME-1];
            
            cv::putText(image,ss.str(),cv::Point(FG_START,floor+((FG_SPACING+FG_HEIGHT/2)*(c+1) )-70),cv::FONT_HERSHEY_SIMPLEX,0.4,history[i].color,1,8,false);
            ss.str("");
            
            int c_start = FG_START;
            int c_floor = floor+(FG_SPACING+ FG_HEIGHT/2 )* (c+1) ;
            
            //DISEGNA IL SOTTOGRAFICO
            
            //disegna la funzione nel grafico
            for (int j = 1; j < HISTORYTIME; j++) {
                cv::line(image, cv::Point(c_start + ((j - 1)*10), c_floor - ((1 - history[i].value[j - 1])*50)),
                        cv::Point(c_start + (j * 10), c_floor - ((1 - history[i].value[j])*50)), history[i].color, 2, 8, 0);
            }

            std::stringstream ss;
            //disegna le linee parallele del grafico
            cv::line(image, cv::Point(c_start, c_floor), cv::Point(c_start + FG_WIDTH, c_floor), cv::Scalar(0, 0, 0), 1, 8, 0);
            ss << 0;
            cv::putText(image, ss.str(), cv::Point(c_start + FG_WIDTH + 1, c_floor), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, false);
            ss.str("");

            cv::line(image, cv::Point(c_start, c_floor - 50), cv::Point(c_start + FG_WIDTH, c_floor - 50), cv::Scalar(0, 0, 0), 1, 8, 0);
            ss << 100;
            cv::putText(image, ss.str(), cv::Point(c_start + FG_WIDTH + 1, c_floor - 50), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, false);
            ss.str("");

            cv::line(image, cv::Point(c_start, c_floor + 10), cv::Point(c_start + FG_WIDTH, c_floor + 10), cv::Scalar(0, 0, 0), 1, 8, 0);
            cv::putText(image, "(q)", cv::Point(c_start + FG_WIDTH + 1, c_floor + 10), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, false);
            
            cv::rectangle(image,cv::Point(c_start,c_floor - 60),cv::Point(FIGURE_WIDTH-10,c_floor + 15), cv::Scalar(0,0,0),2,8,0);
            
            c++;
        }
        cv::imshow("FrequencyGraph",image);
        cv::waitKey(5);
        file<<"\n";
        file.close();
    }
    WM_node * getReleasedInstance( std::string targetInstance ){
        int j = 0;
        //se esiste, prendi l'istanza rilasciata
        while (j < WM->getNodesByInstance(targetInstance).size() &&
                !WM->getNodesByInstance(targetInstance)[j]->isBranchReleased())
            j++;

        WM_node *pivot;

        if (j == 0){
            pivot = WM->getNodesByInstance(targetInstance)[j];
        }
        else{
            pivot = WM->getNodesByInstance(targetInstance)[j - 1];
        }
        
        return pivot;
    }
    //listAll fa un continuo listing e stampa su file
    void listAll(WM_node *node, std::ofstream *file, std::string path){
        
        std::string newPath;
        newPath=path+"_"+node->instance;
        
        for (int i = 0; i < node->son.size(); i++) {
            *file << node->name << "->"
                    << (node->son[i])->instance << ", "
                    << (node->son[i])->rtm << ", "
                    << WM->getInstanceMagnitude(node->son[i]->instance);
            //copia esatta della funzione 
            int j = 0;
            double var;
            char c;
            bool isTrue = true;
            std::string app;
            //mentre il vettore non è finito ed il releaser è vero
            while (j < node->son[i]->releaser.size() && isTrue) {
                var = WMV.get<double>(node->son[i]->releaser[j]);
                //se l'iesimo elemento non è negato
                if (node->son[i]->releaser[j][0] != '-' &&
                        //ed e falso nella memoria allora il releaser è falso
                        var == 0) isTrue = false;
                    //altrimenti se è negato
                else if (node->son[i]->releaser[j][0] == '-') {
                    //apri uno stream
                    std::stringstream ss(node->son[i]->releaser[j]);
                    //scarta il not (ie. il simbolo -)
                    ss >> c>>app;
                    var = WMV.get<double>(app);
                    //se nella memoria è vero allora il releaser è falso
                    if (var == 1) isTrue = false;
                }

                if (isTrue)
                    *file << "\033[0;32m " << node->son[i]->releaser[j] << " \033[0m";
                else
                    *file << "\033[0;31m " << node->son[i]->releaser[j] << " \033[0m";

                j++;
            }

            if (node->son[i]->goalStatus())
                *file << " accomplished\n";
            else
                *file << "\n";
            listAll(node->son[i],file,newPath);
        }
    }
    //printAll stampa su file i periodi ed i path completi
    void printAll(WM_node *node, std::ofstream *file, std::string path){
        
        std::string newPath;
        newPath=path+"_"+node->instance;
        *file<<newPath<<": "<<WM->getInstanceMagnitude(node->instance)<<"|"<<node->rtm<<"\n";
//        if(!node->abstract)
//        {
//            *file<<node->instance<<": "<<WM->getInstanceMagnitude(node->instance)<<"|"<<node->rtm<<"\n";
//        }
        for(int i=0;i<node->son.size();i++)  
            printAll(node->son[i],file,newPath);
    }
    void shift(float *tail, float head){
        for(int i=1;i<HISTORYTIME;i++){
            tail[i-1]=tail[i];
        }
        tail[HISTORYTIME-1]=head;
    }
    void init(float *arr){
        for(int i=0;i<HISTORYTIME;i++){
            arr[i]=1;
        }
    }
    void start(){
        std::ofstream file;
        file.open(fgPath.c_str(),std::ios::trunc);
        file<<"";
        file.close();
        
    }
    void exit(){
        std::cout<<"fg closing window\n";
        cv::destroyWindow("FrequencyGraph");
        std::cout<<"fg removed!\n";
    }
private:
    std::vector< struct WM_graph > history;
    int schemaNumber;
    std::string fgPath;
    int pid;
    //std::vector< std::pair<std::string, float> > sample;
};

#endif	/* MIE_FG_H */

