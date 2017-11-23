#include "ros/ros.h"
#include "std_msgs/String.h"
#include "/home/erik/portaudio/include/portaudio.h"
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>


using namespace std;

#define PATH "/home/erik/Scrivania/Progetto/Progetto/src/speech_seed/"


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
        void *userData){
			
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

int main(int argc,char** argv){
	
	ROS_INFO("SpeechDetector Node start ");
	ros::init(argc,argv,"speech_seed");
	
	//parametri per registrare l'audio
	PaStreamParameters inputParameters;
	PaStream* stream;
	PaError err = paNoError;
	paTestData data,toSend;
	int i;
	int totalFrames;
	int numSamples,numSamplesToSend;
	int numBytes;
	SAMPLE max, val;
	double average;
	int totSeconds;

	

	//inizializzo i parametri per la registrazione
	data.maxFrameIndex = totalFrames = NUM_SECONDS * SAMPLE_RATE; /* Record for a few seconds. */
	data.frameIndex = 0;
	numSamples = totalFrames * NUM_CHANNELS;
	numBytes = numSamples * sizeof (SAMPLE);
	data.recordedSamples = (SAMPLE *) malloc(numBytes); /* From now on, recordedSamples is initialised. */
	
	toSend.maxFrameIndex = NUM_FRAME * NUM_SECONDS * SAMPLE_RATE; /* Record for a few seconds. */
    numSamplesToSend=toSend.maxFrameIndex * NUM_CHANNELS;
	toSend.recordedSamples=(SAMPLE *) malloc(toSend.maxFrameIndex * NUM_CHANNELS * sizeof (SAMPLE));
    toSend.frameIndex = 0;

	err = Pa_Initialize();

	inputParameters.device = Pa_GetDefaultInputDevice(); /* default input device */
	inputParameters.channelCount = 1; /* stereo input */
	inputParameters.sampleFormat = PA_SAMPLE_TYPE;
	inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
	inputParameters.hostApiSpecificStreamInfo = NULL;

	/* Record some audio. -------------------------------------------- */
	err = Pa_OpenStream(
		    &stream,
		    &inputParameters,
		    NULL, /* &outputParameters, */
		    SAMPLE_RATE,
		    FRAMES_PER_BUFFER,
		    paClipOff, /* we won't output out of range samples so don't bother clipping them */
		    recordCallback,
		    &data);

	
	FILE *fid;
	
	stringstream recordedFile;
	stringstream ss;
	//~ stringstream messageFile;
	//~ stringstream lengthFile;
	//~ lengthFile << PATH << "length.txt";
	//~ messageFile << PATH << "message.txt";
	recordedFile << PATH << "recorded.raw";
	//~ fid = fopen(recordedFile.str().c_str(), "wb");
	//~ 
	//~ fclose(fid);
	string command;
	
	ros::NodeHandle nodeH;
	ros::Publisher pubReady = nodeH.advertise<std_msgs::String>("seed_stream",0);
	
	std_msgs::String stringSpeech;
	stringstream speechStream;

	while (ros::ok()) {
		
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
		
		//inizia la registrazione dello stream
		while (j < NUM_FRAME && !audio) {
            audioStop=false;
            while (Pa_IsStreamActive(stream) && !audioStop) {
                Pa_Sleep(10);
            }
            for (int i = 0; i < data.frameIndex; i++) {
                if (data.recordedSamples[i] > th && !listen){
                    listen = true;
                }
                if (data.recordedSamples[i] < th && listen){
                    count++;
                }
                if(listen){
					cout << "listen is true " <<  toSend.frameIndex << endl;
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
           
        //~ err = Pa_StopStream(stream);
           
		if (audio){
		    
			fid = fopen(recordedFile.str().c_str(), "ab");
				
		    if (fid == NULL) {
				cout<< "Could not open file ";
		    } else {
				//scrive i dati su file
		        //~ fwrite(data.recordedSamples, NUM_CHANNELS * sizeof (SAMPLE), data.frameIndex, fid);
		        fwrite(toSend.recordedSamples, NUM_CHANNELS * sizeof (SAMPLE), toSend.frameIndex, fid);
		        fclose(fid);
		        //~ firstWritten = data.frameIndex + 1;
		    }
		    
		        
			//~ data.frameIndex = 0;
			//~ for (i = 0; i < numSamples; i++) 
				//~ data.recordedSamples[i] = 0;

		//}


			//if (hasSpeech) {
			std::string recognized, line;
			std::size_t found;
			bool haveSpeech = false;

	//        std::cout<<"juliusMotor\n";
			
			ss.str("");
			//system("pwd");
			ss << PATH << "scriptSpeechJulius" << " " << PATH << "recorded.raw  2>&1 > /dev/null ";
			system(ss.str().c_str());
			ss.str("");
			ss << PATH << "message.txt";
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
					//~ std::cout << "speech: " << recognized << "\n";
					//pubblicare il messaggio
					speechStream.str("");
					speechStream << "en(\"" << recognized << "\")" ;
					stringSpeech.data = speechStream.str();
					pubReady.publish(stringSpeech);
				}else 
					std::cout<<"speech rejected\n";
			}
			
			
		}
		
		ss.str("");
        ss << PATH << "recorded.raw";
        fid = fopen(ss.str().c_str(), "wb");
	}
	
	//termine del flusso audio libero la memoria
	Pa_Terminate();
	if (data.recordedSamples) /* Sure it is NULL or valid. */
		free(data.recordedSamples);
	
	
	return 0;
}
