/* 
 * File:   mie_IOBehaviour.h
 * Author: hargalaten
 *
 * Created on 16 dicembre 2012, 12.44
 */

#ifndef MIE_IOBEHAVIOUR_H
#define	MIE_IOBEHAVIOUR_H

#include "seed_header.h"



#define RED 16711680
#define YELLOW 16776960
#define GREEN 65280
#define BLUE 255
#define MAX_SONAR_RANGE 3
#define MAX_BLOB_RANGE 10
#define BLOBNEAR 1.5
#define BLOBREACHED 1
#define MAX_SPEED 0.4
#define SAY_SPEED 0.1

class IOBehaviour : public Behaviour{
public:
    void exit(){
        //std::cout<<getName()<<" EXITED\n";
    }
    void start(){
        std::cout<<"SYSTEM: "<<getName()<<" on!\n";
    };
};

class InputStreamBehaviour : public IOBehaviour{
public:   
    InputStreamBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        reqStream="requestStream";
    }
    bool perceptualSchema(){
        //std::cin>>input;
        std::getline(std::cin,input);
        importance=0;
        while(input[input.size()-1]=='!'){
            input.erase(input.size()-1);
            importance++;
        }
        return true;
    }
    void motorSchema(){
        WM_node *son;
        WM_node *req;
        
        //std::cout<< "SYSTEM: " << input << "\n";
        
        pthread_mutex_lock(&memMutex);
        if(WM!=NULL)
            if((req=WM->getNodesByInstance(reqStream)[0]) != NULL){
                son=req->addSon(input);
                son->ltMagnitude+=importance;
            }
            else
                std::cout<<"SYSTEM: no requestStream!\n";
        pthread_mutex_unlock(&memMutex);
        
    }
protected:
    std::string input;
    int importance;
    std::string reqStream;
};

class SonarStreamBehaviour : public IOBehaviour{
public:
    SonarStreamBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
    
        //angolazione dei sonar nel paioneer
        angle.push_back(-90);
        angle.push_back(-50);
        angle.push_back(-30);
        angle.push_back(-10);
        angle.push_back(10);
        angle.push_back(30);
        angle.push_back(50);
        angle.push_back(90);
    }

    bool perceptualSchema(){
        
        std::stringstream ss;
        minSonar=MAX_SONAR_RANGE+1;
        
        robot.Read();
        
        pthread_mutex_lock(&memMutex);
        
        int count;
        count = WMV.get<double>("sonar.count");
        
        for(int i=0;i<count;i++){
            ss<<"sonar"<<i;
            WMV.set<double>(ss.str(),sonar[i]);
            //occhio che il sonar può uscire negativo -.-"
            if(sonar[i]<minSonar && sonar[i]>0) minSonar=sonar[i];
            ss.str("");  
        }
        
        WMV.set<double>("sonar.globalMin",minSonar);
        
        pthread_mutex_unlock(&memMutex);
        
        return true;
    }
    void motorSchema(){
        
        pthread_mutex_lock(&memMutex);
        //aggiorno il ritmo in base al sonar più basso
        updateRtm(minSonar,MAX_SONAR_RANGE,0);
        pthread_mutex_unlock(&memMutex);
    
    }
    void start(){
        int count;
        std::stringstream ss;
        pthread_mutex_lock(&memMutex);   
        count=angle.size();
        WMV.set<double>("sonar.count",count);
        for(int i=0;i<count;i++){
            ss<<"sonar"<<i<<".sin";
            WMV.set<double>(ss.str(),sin(dtor(angle[i])));
            ss.str("");
            ss<<"sonar"<<i<<".cos";
            WMV.set<double>(ss.str(),cos(dtor(angle[i])));
            ss.str("");         
        }
        pthread_mutex_unlock(&memMutex);
    };
private:
    std::vector<double> angle;
    double minSonar;
};

class EngineStreamBehaviour : public IOBehaviour{
public:
    EngineStreamBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
    }
    bool perceptualSchema(){
        
        robot.Read();
        
        collide=engine.GetStall();
        
        pthread_mutex_lock(&memMutex);
        double fsCount,tsCount;
//        std::cout<<"winner: "<<WM->getNodesByInstance(this->getInstance())[0]->winner<<"\n";
        fs=WMV.get<double>("engine.fs");
        ts=WMV.get<double>("engine.ts");
        fsCount=WMV.get<double>("engine.fs.count");
        tsCount=WMV.get<double>("engine.ts.count");
        if(fsCount!=0){
            fs/=fsCount;
            WMV.set<double>("engine.fs",fs);
        }
        if(tsCount!=0){
            ts/=tsCount;
            WMV.set<double>("engine.ts",ts);
        }
        WMV.set<double>("engine.fs.count",0);
        WMV.set<double>("engine.ts.count",0);
        //azzera il vincitore
        WM->getNodesByInstance(this->getInstance())[0]->winner="";
        
        //perception["engine.winner"]=0;
        
        //normalizza
//        if(command!=0){
//            perception["engine.fs"]/=command;
//            perception["engine.ts"]/=command;
//            perception["engine.command"]=0;
//            fs=perception["engine.fs"];
//            ts=perception["engine.ts"];
//        }
        
        if(collide)
            WMV.set<double>("engine.collide",1);
        else
            WMV.set<double>("engine.collide",0);
        
        //predizione sulla futura posizione del robot
        WMV.set<double>("engine.x",engine.GetXPos()+(cos(dtor(ts))*fs));
        WMV.set<double>("engine.y",engine.GetYPos()+(sin(dtor(ts))*fs));
        WMV.set<double>("engine.yaw",rtod(engine.GetYaw()+atan2(sin(dtor(ts)),cos(dtor(ts)))));
        
        pthread_mutex_unlock(&memMutex);
        
        return true;
    }
    void motorSchema(){
        
//        std::cout<<"ENGINE: ts: "<<ts<<",fs: "<<fs<<"\n";
        
        //se non sono andato a sbattere eseguo il comando
        if(!collide)
            engine.SetSpeed(fs,dtor(ts));
        //altrimenti vado indietro fino a liberarmi
        else
            engine.SetSpeed(-0.1,0);
        
        pthread_mutex_lock(&memMutex);
        
        double minSonar=WMV.get<double>("sonar.globalMin");
        
        if(fs!=0)
            updateRtm(minSonar/fs,MAX_SPEED,0);
        
        pthread_mutex_unlock(&memMutex);
    }
    void exit(){
        pthread_mutex_lock(&memMutex);
        engine.SetSpeed(0,0);
        pthread_mutex_unlock(&memMutex);
        //std::cout<<getName()<<" EXITED\n";
    }
protected:
    bool collide;
    double fs;
    double ts;

};

class BlobStreamBehaviour : public IOBehaviour{
public:
    BlobStreamBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
    }
    bool perceptualSchema(){
        
        int i;
        std::stringstream ss;
        std::string colorName;
        min=15;
        
        robot.Read();
        
        pthread_mutex_lock(&memMutex);
        
        //azzera la presenza dei colori ad ogni ciclo
        WMV.set<double>("red.present",0);
        WMV.set<double>("blue.present",0);
        WMV.set<double>("green.present",0);
        WMV.set<double>("yellow.present",0);
        
        WMV.set<double>("red.near",0);
        WMV.set<double>("blue.near",0);
        WMV.set<double>("green.near",0);
        WMV.set<double>("yellow.near",0);
        
        WMV.set<double>("red.reached",0);
        WMV.set<double>("blue.reached",0);
        WMV.set<double>("green.reached",0);
        WMV.set<double>("yellow.reached",0);
        
        WMV.set<double>("blob.count", blob.GetCount());
        
        for(i=0;i<blob.GetCount();i++)
        {
            //esporta le proprietà dei blob rilevati
            ss<<"blob["<<i<<"]";
            WMV.set<double>(ss.str()+".color", blob[i].color);
            WMV.set<double>(ss.str()+".range", blob[i].range);
            WMV.set<double>(ss.str()+".x", blob[i].x);
            WMV.set<double>(ss.str()+".y", blob[i].y);
            
            
            //trova il blob minimo
            if(blob[i].range<min)
                min=blob[i].range;
            
            //rileva il volore del blob
            if(blob[i].color==RED)
                colorName="red";
            else if(blob[i].color==BLUE)
                colorName="blue";
            else if(blob[i].color==GREEN)
                colorName="green";
            else if(blob[i].color==YELLOW)
                colorName="yellow";
            
            //il colore è presente
            WMV.set<double>(colorName+".present",1);
            
            //il colore è vicino
            if(blob[i].range <= BLOBNEAR)
                WMV.set<double>(colorName+".near",1);
            
            //il colore è stato raggiunto
            if(blob[i].range <= BLOBREACHED)
                WMV.set<double>(colorName+".reached",1);
            
            ss.str("");
                
        }
        
        pthread_mutex_unlock(&memMutex);
        
        return true;
        
    }
    void motorSchema(){
        
        pthread_mutex_lock(&memMutex);
        updateRtm(min,MAX_BLOB_RANGE,1);
        pthread_mutex_unlock(&memMutex);
        
    }
    void start(){
        pthread_mutex_lock(&memMutex);

        //non so perche ma non funziona getWidth
//        perception["blob.width"]=blob.GetWidth();
        WMV.set<double>("blob.width",80.0);

        pthread_mutex_unlock(&memMutex);
    }
    void exit(){
        pthread_mutex_lock(&memMutex);
        
        //azzera la presenza dei colori
        WMV.set<double>("red.present",0);
        WMV.set<double>("blue.present",0);
        WMV.set<double>("green.present",0);
        WMV.set<double>("yellow.present",0);
        
        WMV.set<double>("red.near",0);
        WMV.set<double>("blue.near",0);
        WMV.set<double>("green.near",0);
        WMV.set<double>("yellow.near",0);
        
        WMV.set<double>("red.reached",0);
        WMV.set<double>("blue.reached",0);
        WMV.set<double>("green.reached",0);
        WMV.set<double>("yellow.reached",0);
        
        pthread_mutex_unlock(&memMutex);
        
        //std::cout<<getName()<<" EXITED\n";
    }
protected:
    double min;
};

#define SAMPLE_RATE  (44100)
#define FRAMES_PER_BUFFER (512)
#define NUM_SECONDS     (0.3)
#define NUM_CHANNELS    (1)
/* #define DITHER_FLAG     (paDitherOff) */
#define DITHER_FLAG     (0) 

#define WRITE_TO_FILE   (1)
#define PA_SAMPLE_TYPE  paInt16
typedef short SAMPLE;
#define SAMPLE_SILENCE  (0)
#define PRINTF_S_FORMAT "%d"

#define NUM_FRAME (30)
typedef struct {
    int frameIndex; /* Index into sample array. */
    int maxFrameIndex;
    SAMPLE *recordedSamples;
}
paTestData;

bool audioStop;

static int recordCallback(const void *inputBuffer, void *outputBuffer,
        unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo* timeInfo,
        PaStreamCallbackFlags statusFlags,
        void *userData) 
    {
        paTestData *data = (paTestData*) userData;
        const SAMPLE *rptr = (const SAMPLE*) inputBuffer;
        SAMPLE *wptr = &data->recordedSamples[data->frameIndex * NUM_CHANNELS];
    long framesToCalc;
    long i;
    int finished;
    unsigned long framesLeft = data->maxFrameIndex - data->frameIndex;

    (void) outputBuffer; /* Prevent unused variable warnings. */
    (void) timeInfo;
    (void) statusFlags;
    (void) userData;
    
//    std::cout<<"callback\n";

    if (framesLeft < framesPerBuffer) {
        framesToCalc = framesLeft;
//        finished = paComplete;
        audioStop=true;
        finished = paContinue;
    } else {
        framesToCalc = framesPerBuffer;
        finished = paContinue;
    }

    if (inputBuffer == NULL) {
        for (i = 0; i < framesToCalc; i++) {
            *wptr++ = SAMPLE_SILENCE; /* left */
            if (NUM_CHANNELS == 2) *wptr++ = SAMPLE_SILENCE; /* right */
        }
    } else {
        for (i = 0; i < framesToCalc; i++) {
            *wptr++ = *rptr++; /* left */
            if (NUM_CHANNELS == 2) *wptr++ = *rptr++; /* right */
        }
    }
    data->frameIndex += framesToCalc;
    return finished;
}


class AudioStreamBehaviour : public IOBehaviour{
public:
    AudioStreamBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
    }

    bool perceptualSchema(){
        
        
        int j=0;
        toSend.frameIndex=0;
	bool listen, audio;
        listen=false;
        audio=false;
        int th = 1300;
        int count=0;

        
        for (int i = 0; i < numSamples; i++)
            data.recordedSamples[i] = 0;
        data.frameIndex = 0;

        //err = Pa_StartStream(stream);
        //printf("\n=== Now recording!! Please speak into the microphone. ===\n");
        //fflush(stdout);
        
        

        while (j < NUM_FRAME && !audio) {
            audioStop=false;
            while (Pa_IsStreamActive(stream) && !audioStop) {
                Pa_Sleep(10);
            }
//            err = Pa_StopStream(stream);
            for (int i = 0; i < data.frameIndex; i++) {
                if (data.recordedSamples[i] > th && !listen){
//                    std::cout << "listen!\n";
                    listen = true;
                }
                if (data.recordedSamples[i] < th && listen){
                    count++;
                }
                if(listen){
                        toSend.recordedSamples[toSend.frameIndex] = data.recordedSamples[i];
                        toSend.frameIndex++;
                }
                data.recordedSamples[i] = 0;
            }
            if (count == data.frameIndex)
                audio = true;
            if(listen)
                j++;
            count=0;
            data.frameIndex = 0;
            err = Pa_StartStream(stream);
        }
        

//        std::cout<<"audio: "<<audio<<"\n";
        
        
        
        return true;
    }
    void motorSchema(){
        
        pthread_mutex_lock(&memMutex);
        
        buffer->push_back(toSend);
        
        setRtm(0.01);
        
        pthread_mutex_unlock(&memMutex);
        
        toSend.recordedSamples=(SAMPLE *) malloc(toSend.maxFrameIndex * NUM_CHANNELS * sizeof (SAMPLE));
        toSend.frameIndex = 0;
//        for (int i = 0; i < numSamplesToSend; i++)
//            toSend.recordedSamples[i] = 0;
//        toSend.frameIndex = 0;
        
        
    
    }
    void start(){
        
        
        err = paNoError;
        
        data.maxFrameIndex = totalFrames = NUM_SECONDS * SAMPLE_RATE; /* Record for a few seconds. */
	data.frameIndex = 0;
	numSamples = totalFrames * NUM_CHANNELS;
	numBytes = numSamples * sizeof (SAMPLE);
	data.recordedSamples = (SAMPLE *) malloc(numBytes); /* From now on, recordedSamples is initialised. */
        
        toSend.maxFrameIndex = NUM_FRAME * NUM_SECONDS * SAMPLE_RATE; /* Record for a few seconds. */
        numSamplesToSend=toSend.maxFrameIndex * NUM_CHANNELS;
	toSend.frameIndex = 0;
	toSend.recordedSamples = (SAMPLE *) malloc(toSend.maxFrameIndex * NUM_CHANNELS * sizeof (SAMPLE)); /* From now on, recordedSamples is initialised. */
        
        
	err = Pa_Initialize();

	inputParameters.device = Pa_GetDefaultInputDevice(); /* default input device */
	//inputParameters.channelCount = 2; /* stereo input */
        inputParameters.channelCount = NUM_CHANNELS; 
	inputParameters.sampleFormat = PA_SAMPLE_TYPE;
	inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
	inputParameters.hostApiSpecificStreamInfo = NULL;
        
        err = Pa_OpenStream(
		    &stream,
		    &inputParameters,
		    NULL, /* &outputParameters, */
		    SAMPLE_RATE,
		    FRAMES_PER_BUFFER,
		    paClipOff, /* we won't output out of range samples so don't bother clipping them */
		    recordCallback,
		    &data);
        
//        fid = fopen("recorded.raw", "wb");
//	fclose(fid);
        
        
        pthread_mutex_lock(&memMutex);
        
        buffer=new std::vector<paTestData>;
        
        WMV.set("audioBuffer", (void*) buffer);
        WMV.set<double>("audioChannels", 1);
        
        pthread_mutex_unlock(&memMutex);
        
    };
    void exit(){
        pthread_mutex_lock(&memMutex);
        buffer=NULL;
        pthread_mutex_unlock(&memMutex);
        Pa_Terminate();
        if (data.recordedSamples) /* Sure it is NULL or valid. */
	    free(data.recordedSamples);
    }
private:
    PaStreamParameters inputParameters;
	PaStream* stream;
	PaError err;
	paTestData data,toSend;
	int totalFrames;
	int numSamples, numSamplesToSend;
	int numBytes;
	SAMPLE max, val;
	double average;
	int lastSeconds;
	int totSeconds;
        FILE *fid;
        std::vector<paTestData> *buffer;
};

class VocalStreamBehaviour : public IOBehaviour{
public:   
    VocalStreamBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        toSay="";
    }
    bool perceptualSchema(){
        
        bool haveToSay=false;
        std::string fromMem;
        
        pthread_mutex_lock(&memMutex);
        setRtm(0.1);
        
        fromMem=WMV.get<std::string>("vocalStream.say");
        lan=WMV.get<std::string>("vocalStream.language");
        
        WMV.set<double>("vocalStream.say.count", 0);
        WMV.set<double>("vocalStream.language.count", 0);
        pthread_mutex_unlock(&memMutex);
        
        if(fromMem!=toSay && fromMem!=""){
            toSay=fromMem;
            haveToSay=true;
        }
        
        return haveToSay;
    }
    void motorSchema(){
        std::stringstream toSystem;
        
//        std::cout<<toSay<<"\n";
        
        if(lan=="it")
            toSystem<<"espeak -v mb-it3 -s 130 \""<<toSay<<"\" 2> /dev/null";
        else if(lan=="en")
            toSystem<<"espeak -v mb-en1 -s 130 \""<<toSay<<"\" 2> /dev/null";
        
        
        system(toSystem.str().c_str());
        
        pthread_mutex_lock(&memMutex);
        
        WMV.set<double>("say("+lan+"(\""+toSay+"\")).done", 1);
        pthread_mutex_unlock(&memMutex);
        
    }
protected:
    std::string toSay;
    std::string lan;
};

class SayBehaviour : public IOBehaviour{
public:   
    SayBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(SAY_SPEED);
        
        std::stringstream lexical,lan;
        char c;
        
        int lstart=this->getInstance().find("\"");
        int lend=this->getInstance().find("\"",lstart+1);
        
        lexical<<instance2vector(this->getInstance())[1];
        
        lexical>>c;
        while(c!='('){
            lan<<c;
            lexical>>c;
        }
        
        language=lan.str();
        
        toSay=this->getInstance().substr(lstart+1,lend-(lstart+1));
    }
    bool perceptualSchema(){
        std::string saying;
        
        pthread_mutex_lock(&memMutex);
        setRtm(SAY_SPEED);
        
        //prendo la parola che sto dicendo
        saying=WMV.get<std::string>("vocalStream.say");
        
        //se ho raggiunto il goal stacco tutto
        if(dead()) return false;
        if(WM->getNodesByInstance(this->getInstance())[0]->goalStatus()){
            remove(WM->getNodesByInstance(this->getInstance())[0]);
        }
        
        pthread_mutex_unlock(&memMutex);
        
        //se la parola che sto dicendo non è quella che devo dire rilascio
        return (saying!=toSay);
    }
    void motorSchema(){
        
        
        bool result;
        
        
//        std::cout<<lexical.str()<<"\n";
        
        
        //toSay.erase(toSay.size());
//        std::replace( toSay.begin(), toSay.end(), '.', ' ');
        //std::cout<<"SYSTEM: "<<toSay<<"\n";
        pthread_mutex_lock(&memMutex);
        send("vocalStream","vocalStream.say",toSay);
        send("vocalStream","vocalStream.language",language);
//        WMV.set("vocalStream.say",(std::string) toSay);
//        WMV.set("vocalStream.language",(std::string) language);
        
//        if(dead()) return;
//            
//        remove(WM->getNodesByInstance(this->getInstance())[0]);
        
        pthread_mutex_unlock(&memMutex);
    }
    void start(){
        
    }
    void exit(){
        pthread_mutex_lock(&memMutex);
        WMV.set<double>(this->getInstance() + ".done", 0);
        pthread_mutex_unlock(&memMutex);
    }
protected:
    bool speacking;
    std::string toSay, language;
};


#endif	/* MIE_IOBEHAVIOUR_H */

