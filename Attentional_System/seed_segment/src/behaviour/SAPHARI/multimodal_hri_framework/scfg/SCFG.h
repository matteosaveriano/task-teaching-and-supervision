/* 
 * File:   SCFG.h
 * Author: hargalaten
 *
 * Created on 12 settembre 2014, 18.03
 */

#ifndef SCFG_H
#define	SCFG_H

#include "../../../../seed_header.h"
#include "../AlfredDialogueManager/POMDP.h"


struct rule{
	std::string head;
	std::vector<std::string> body;
	double confidence;
} ;

struct parseTree{
	bool used;
	double p;
} ;

class SCFG{
	
	std::vector<rule> rules;
	
	public:

		SCFG(std::string);
		std::vector<rule> getRules();
		std::vector<Observ> is_inGrammar(std::string);
		
	private:
		
		bool addRule(std::string);
		std::vector<std::string> getTokenWord(std::string);
    };
    
    

SCFG::SCFG(std::string fileName){
	
	std::string line;
	
	//apri file
	std::ifstream file;
	file.open(fileName.c_str());
	if (file.is_open()){
//            std::cout<<"reading file\n";
		//read line
		while (getline(file,line)){
			//add rule in grammar
			if (line[0] != '#') //comment line
				addRule(line);
		} 
	}
		
}


bool SCFG::addRule(std::string line){
	
	if (line != ""){
		rule temp;
		
		size_t h = line.find(" ->");
		size_t separator = line.find(",");
		
		temp.head = line.substr(0,h);
		//~ cout << "aggiunto " << temp.head << endl;
		
		if (separator != std::string::npos){
			//esiste il separatore
			temp.body.push_back(line.substr(h+4,separator-(h+4)));
			temp.body.push_back(line.substr(separator+1,line.find(" ",separator+1)-(separator+1)));
//			std::cout << "body 0 " << temp.body[0] << std::endl;
//			std::cout << "body 1 " << temp.body[1] << std::endl;
		} else {
			temp.body.push_back(line.substr(h+4,line.find(" ",h+4)-(h+4)));
//			std::cout << "body 0 " << temp.body[0] << std::endl;
		}
		
		//get confidence
		size_t start_prob = line.find("[");
		size_t end_prob = line.find("]");
		if (start_prob != std::string::npos){
			std::string confidence = line.substr(start_prob+1,end_prob-(start_prob-1));
			temp.confidence = atof(confidence.c_str());
//			std::cout << "prob = " << temp.confidence << std::endl;
		}
		rules.push_back(temp);
		
	} else 
		return false;
	
	return true;
}

vector<rule> SCFG::getRules(){
	return rules;
}


vector<std::string> SCFG::getTokenWord(std::string word){

	vector<std::string> token;
	std::string delimiter = " ";
	
	size_t n = word.find(delimiter);
	size_t oldN = n + 1;
	
	token.push_back(word.substr(0,n));
	
	while (n != std::string::npos){ 
		n = word.find(delimiter,oldN);
		if (n != std::string::npos)
			token.push_back(word.substr(oldN,n-oldN));
		else 
			token.push_back(word.substr(oldN));
		oldN = n + 1;
	}

	return token;
	
}


std::vector<Observ> SCFG::is_inGrammar(std::string word){
	vector<std::string> words = getTokenWord(word);

	int n = words.size();
	int r = rules.size();
	parseTree treD[n][n][r];
	//init
	for (int i = 0; i < n; i++){
		for (int j = 0; j < r; j++){
			if (rules[j].body[0] == words[i]){
//                std::cout << j << " " << words[i] << std::endl;
				treD[i][0][j].used = true;
				treD[i][0][j].p = rules[j].confidence;
			} else {
				treD[i][0][j].used = false;
			}
			for (int k = 1; k < n; k++)
				treD[i][k][j].used = false;
		}
	}
	 
	//check
	for (int i = 1; i < n; i++){ //length of span
		for (int j = 0; j < n-i; j++){ //start of span
			for (int k = 0; k < i; k++){ //partition of span
			
				
				for (int ii=0; ii < r; ii++){
					if (treD[j][k][ii].used){
						rule t;
						t.body.push_back(rules[ii].head);
						for (int jj=0; jj < r; jj++){
							if (treD[j+k+1][i-k-1][jj].used){
								t.body.push_back(rules[jj].head);
								//cout << "esamino " << t.body[0] << " " << t.body[1] << " " << i << " " << j << " " << k << endl;
								for (int y=0; y < r; y++){
									if (rules[y].body.size() > 1){
										if ((rules[y].body[0].compare(t.body[0]) == 0) && (rules[y].body[1].compare(t.body[1]) == 0)){
											treD[j][i][y].used = true;
											treD[j][i][y].p = rules[y].confidence * treD[j][k][ii].p * treD[j+k+1][i-k-1][jj].p;
//											std::cout << "regola " << rules[y].head << " -> " << rules[y].body[0] << " " << rules[y].body[1] << " " << i << " " << j << " " << y << std::endl;
										} 
									}
								}
								if (t.body.size() > 1){
									t.body.erase(t.body.begin()+1);
								}
							}
						}
						if (t.body.size() > 0){
							t.body.erase(t.body.begin(),t.body.end());
						}
					}
				}
				
				
			}
		}
	}
	
	bool inGrammar = false;
	//double max_p = -1;
	int iObs; //max_index
	
        std::vector<Observ> obs;
        
	for (int s = 0; s < r; s++){
		if (treD[0][n-1][s].used){
			if (rules[s].head.compare("S") == 0){
				inGrammar = true;
                                
                                std::cout<<"intention: "<<rules[s].body[0]<<" p:"<<treD[0][n-1][s].p<<"\n";
                                
                                double pValue=treD[0][n-1][s].p;
                                Observ swap(rules[s].body[0], pValue);
                                
                                obs.push_back(swap);
                                iObs=obs.size()-1;
                                
                                while(iObs>0 && pValue>obs[iObs-1].GetP()){
                                    obs[iObs]=obs[iObs-1];
                                    iObs--;
                                }
                                
                                obs[iObs]=swap;
                                
			}
		} 
	}
	
//	std::cout << "intention's rule " << rules[max_index].head << " -> " << rules[max_index].body[0] << " " << rules[max_index].body[1] << std::endl;
	
	return obs;
}


//
//int main(int argc, char* argv[]){
//
//	ros::init(argc,argv,"fusion");
//	
//	SCFG g("grammar");
//	if (g.is_inGrammar("gesture_take speech_give obj"))
//		cout << "is in grammar" << endl;
//	else
//		cout << "is not in grammar" << endl;
//	//while (ros::ok() ){
//	
//	 	
//    	
//	//}
//	return 0;
//}

#endif	/* SCFG_H */

