/* 
 * File:   juliusAudioRecognizer.h
 * Author: hargalaten
 *
 * Created on 21 maggio 2014, 12.04
 */

#ifndef JULIUSAUDIORECOGNIZER_H
#define	JULIUSAUDIORECOGNIZER_H

#include "seed_header.h"
#include "mie_IOBehaviour.h"

class JuliusAudioRecognizerBehaviour : public Behaviour {
public:

    JuliusAudioRecognizerBehaviour(std::string instance) {
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        WMV.set<double>(instance + ".n", 2);
        WMV.set<std::string>(instance + ".best1.command", "");
        WMV.set<std::string>(instance + ".best2.command", "");
        path = "/home/hargalaten/Scrivania/ros_sources/roseed/src/SEED/";

    }

    bool perceptualSchema() {

        bool audio = false;
        

        pthread_mutex_lock(&memMutex);

        buffer = WMV.get<std::vector<paTestData> *>("audioBuffer");

        if (buffer != NULL && !buffer->empty()) {

            toSend = buffer->front();
            //            std::cout << "print: ";
            //            for (int i = 0; i < toSend.frameIndex; i += 1000)
            //                std::cout << toSend.recordedSamples[i] << ",";
            //            std::cout << "\n";

            buffer->erase(buffer->begin());
            audio = true;
            num_channels=WMV.get<double>("audioChannels");
        }

        pthread_mutex_unlock(&memMutex);

        return audio;
    }

    void motorSchema() {

        //        pthread_mutex_lock(&memMutex);
        //        
        //        pthread_mutex_unlock(&memMutex);

        std::stringstream ss;

        c++;

        ss << path << "recorded.raw";

        fid = fopen(ss.str().c_str(), "ab");
        if (fid == NULL) {
            printf("Could not open file.");
        } else {
            fwrite(toSend.recordedSamples, num_channels * sizeof (SAMPLE), toSend.frameIndex, fid);
            fclose(fid);
        }


        std::string recognized, line;
        std::size_t found;
        bool haveSpeech = false;

//        std::cout<<"juliusMotor\n";
        
        ss.str("");
        //system("pwd");
        ss << path << "scriptSpeechJulius" << " " << path << "recorded.raw " << c << " 2>&1 > /dev/null ";
        system(ss.str().c_str());
        ss.str("");
        ss << path << "message.txt";
        std::ifstream response(ss.str().c_str());
        //        getline(response, recognized);
        
        while (!haveSpeech && getline(response, line)) {
            std::cout<<"jul: "<<line<<"\n";
            found = line.find("sentence1:");
            if (found != std::string::npos)
                haveSpeech = true;
        }

//        std::cout << "rec: " << recognized << "\n";

        if (haveSpeech) {
            int start = line.find(">");
            int end = line.find("</", start);
            recognized = line.substr(start+2,end-start-3);
            
            std::transform(recognized.begin(), recognized.end(), recognized.begin(), ::tolower);
            //std::replace( recognized.begin(), recognized.end(), ' ', '.');
            std::size_t recfind=recognized.find("computer",0);
            if(recfind!=std::string::npos)
            {
                std::cout << "speech: " << recognized << "\n";
                pthread_mutex_lock(&memMutex);
                if (dead()) return;
                
                //rimuovo gli speech vecchi
                while(WM->getNodesByInstance(this->getInstance())[0]->son.size() > 0)
                    remove(WM->getNodesByInstance(this->getInstance())[0]->son[0]);
                
                WM->getNodesByInstance(this->getInstance())[0]->addSon("en(\"" + recognized + "\")");

                WMV.set<std::string>(this->getInstance() + ".best1.command", recognized);
                WMV.set<double>(this->getInstance() + ".best1.p", 0.9);

                WMV.set<std::string>(this->getInstance() + ".best2.command", "NONE");
                WMV.set<double>(this->getInstance() + ".best2.p", 0.1);

                pthread_mutex_unlock(&memMutex);
            }
            else std::cout<<"speech rejected\n";
        }

        //calcola la lunghezza dell'audio del file
        ss.str("");
        ss << "ecalength " << path << "message.flac | grep -o 0m.*s >" << path << "length.txt";
        system(ss.str().c_str());


        ss.str("");
        ss << path << "lenght.txt";
        std::ifstream audiolength(ss.str().c_str());
        std::string length;
        getline(audiolength, length);

        ss.str("");
        ss << path << "recorded.raw";
        fid = fopen(ss.str().c_str(), "wb");
    }

    void exit() {

        //std::cout<<this->getInstance()<<" EXITED\n";
    }

    void start() {
        std::stringstream ss;
        ss << path << "recorded.raw";
        fid = fopen(ss.str().c_str(), "wb");
        
        std::cout<<ss.str().c_str()<<"\n";
        
        fclose(fid);
        c = 0;
        std::cout << this->getInstance() << " STARTED\n";
    };
private:
    FILE *fid;
    paTestData toSend;
    std::vector<paTestData> *buffer;
    int num_channels;
    int c;
    std::string path;
};
#endif	/* JULIUSAUDIORECOGNIZER_H */

