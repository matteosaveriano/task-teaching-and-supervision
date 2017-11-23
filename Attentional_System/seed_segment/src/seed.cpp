/* 
 * File:   main.cpp
 * Author: hargalaten
 *
 * Created on 4 dicembre 2012, 13.59
 */

#include "seed_header.h"
#include <cstdlib>


WM_node *WM=new WM_node("alive",NULL);
WM_varMap WMV;
pthread_mutex_t memMutex = PTHREAD_MUTEX_INITIALIZER;

std::string winner("none");
    
std::string SYS_HOME_PATH;//("/home/prisma-airobots");
std::string SEED_HOME_PATH;//("/home/prisma-airobots/catkin_ws/src/seed");
    
    
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
        commLine << "addr2line %p -e " << SYS_HOME_PATH << "/home/dhrikarl/Desktop/Coffee_demo/seed_segment/bin/seed_segment";
        sprintf(syscom, commLine.str().c_str() , trace[i]); //last parameter is the name of this app
        //std::cout<<syscom<<"\n";
        system(syscom);
    }
   
    exit(1);
}




/*
 * 
 */
int main(int argc, char** argv) {


	ros::init(argc,argv,"seed_node");
  
  ros::NodeHandle nh;
  //std::string eclpse_path;
  //nh.param("/seed/eclipse_prolog_path", eclpse_path, std::string("eclipseclp/") );

  //avvia ECLIPSE
  //const char* home = getenv("SEED_SRC");
  //std::string path(home);
  //std::cout<<"home: "<<home<<"\n";

  SEED_HOME_PATH = ros::package::getPath("seed_segment");
  std::cout << SEED_HOME_PATH << std::endl;

  SYS_HOME_PATH = getenv("HOME");
  std::cout << SYS_HOME_PATH << std::endl;
  
  std::string eclpse_path = SEED_HOME_PATH + "/eclipseclp";
  ec_set_option_ptr(EC_OPTION_ECLIPSEDIR, (void *) eclpse_path.c_str());
  //ec_set_option_ptr(EC_OPTION_ECLIPSEDIR, (void *)"/home/prisma-airobots/catkin_ws/src/seed/eclipseclp");
  ec_init();
  
  //leggi la long time memory
  //std::string seed_path = ros::package::getPath("seed");
  std::string LTM_path = "[\'" + SEED_HOME_PATH + "/LTM.prolog" + "\']";
  //std::cout<<"seedpath: "<<seed_path<<"\n";

  ec_post_string(LTM_path.c_str());
  //ec_post_string("[\'/home/prisma-airobots/catkin_ws/src/seed/LTM.prolog\']");
  EC_resume();
  
  signal(SIGSEGV,faultHandler);
  
  pthread_mutex_lock(&memMutex);
  WMV.set<double>("TRUE",1);
  WMV.set<double>("FALSE",0);
  pthread_mutex_unlock(&memMutex);
  AliveBehaviour *Alive=new AliveBehaviour("alive");
  //inizializza memoryStream
  pthread_t thread_alive;
  //avvia il thread
  int memoryStream = pthread_create(&thread_alive, NULL,execution, (void *)Alive);
  //aspetta il thread
  pthread_join(thread_alive, NULL);
  
  return 0;  
}

EC_word writeList(std::vector<std::string> vec){
    EC_word l=nil();
    
    for(int i=0;i<vec.size();i++){
        char *cstr = new char[vec[i].length() + 1];
        strcpy(cstr, vec[i].c_str());
        l=list(EC_atom(cstr),l);
        delete [] cstr;
    }
    return l;
}

std::vector<std::string> readList(EC_word list){
    char *buf;
    std::vector<std::string> app;
    EC_word head,tail;
    if(list.is_nil()){
        list.is_list(head,tail);
        head.is_string(&buf);
        //std::cout<<"read: "<<buf<<"\n";
        app=readList(tail);
        app.push_back(std::string(buf));
    }
    return app;
}

void printList(EC_word list)
{
    char *buf;
    EC_word head,tail;
    
    list.is_list(head,tail);
    head.is_string(&buf);
    if(tail.is_nil())
    {
        printf(" %s", buf);
        printList(tail);
    }
    else printf(" %s\n", buf); 
}

void printSchemaList(EC_word list)
{
    double d;
    EC_word head,tail,name,abstract,rtm,releaserList,s_tail,releaser;
    
    
    list.is_list(head,tail);
    
    head.is_list(name,s_tail);
    s_tail.is_list(abstract,s_tail);
    s_tail.is_list(rtm,s_tail);
    s_tail.is_list(releaserList,s_tail);
    
    
    
    std::cout<<"        name: "<<functor2string(name)<<"\n";
    abstract.is_double(&d);
    std::cout<<"        abstract: "<<d<<"\n";
    rtm.is_double(&d);
    std::cout<<"        rhythm: "<<d<<"\n";
    std::cout<<"        releaser: ";
    while(releaserList.is_nil())
    {
        releaserList.is_list(releaser,releaserList);
        //releaser.is_string(&buf);
        
        std::cout<<functor2string(releaser)<<" ";
    }
    std::cout<<"\n\n";
    if(tail.is_nil())
        printSchemaList(tail);
    
    
}

float r2d( float rad ) {
    return rad*180.0/M_PI;
}   

float d2r( float deg ) {
    return deg*M_PI/180.0;
}

//trasforma in stringa un generico funtore restituito da ECLIPSE
std::string functor2string(EC_word f){
    char* buf;
    char* st;
    long stl;
    std::string result;
    std::stringstream ss;
    EC_word arg, parHead,parTail;
    EC_functor fun;
    EC_atom atm;
    double d;
    int i;
    
    //se il funtore ha arietà maggiore di zero
    if(f.arity()!=0) 
    {
        //estrai in nome del funtore
        f.functor(&fun);
        buf=fun.name();
        result.append(buf);
        
        //se è un funtore punto (ie. del tipo schema.attributo)
        if(result=="." && f.arity()==2)
        {
            //svuota la stringa
            result="";
            //prendi lo schema
            f.arg(1,arg);
            result.append(functor2string(arg));
            //inseriscilo nella stringa seguito dal punto
            result=result+".";
            //prendi il metodo
            f.arg(2,arg);
            result.append(functor2string(arg));      
        }
        
        else if(result=="-" && f.arity()==1){
            //prendi il parametro ed aggiungilo alla stringa
            f.arg(1,arg);
            //arg.is_string(&buf);
            result.append(functor2string(arg));   
        }
        //altrimenti è uno schema avente parametri (eg. goto(X))
        else
        {
            //aggiungi una parentesi aperta
            result.append("(");
            //per ogni argomento
            for(i=1;i<=f.arity();i++)
            {
                //inserisci l'argomento nella stringa
                f.arg(i,arg);
                
                
                //se l'i-esimo argomento è una lista
                if (!arg.is_list(parHead, parTail)) {
                    //mantieni la sintasi di lista
                    result.append("[");
                    result.append(functor2string(parHead));
                    //std::cout<<"head: "<<functor2string(parHead)<<"\n";
                    while (parTail.is_nil()) {
                        result.append(",");
                        parTail.is_list(parHead, parTail);
                        result.append(functor2string(parHead));
                    }
                    result.append("],");
                    //std::cout << "list: " << result << "\n";
                }
                //altrimenti è un funtore
                else
                    result=result+functor2string(arg)+",";
                
                //sleep(0.1);
            }
            result.replace(result.size()-1,1,")");
            
            //std::cout<<"end functor: "<<result<<"\n";
        }
    }
    //altrimenti non ha argomenti
    else 
    {
        //se è un atomo
        if(!f.is_atom(&atm)){
//            std::cout<<"è atomo: "<<atm.name()<<"\n";
            result.append(atm.name());
        }
        //altrimenti, se è una stringa
        else if(!f.is_string(&buf)){
//            std::cout<<"è stringa: "<<buf<<"\n";
            std::string element(buf);
            
            //se la stringa è un TRUE o FALSE aggiungilo
            if(element=="TRUE" || element=="FALSE")
                result.append(element);
            //altrimenti aggiungi gli apici
            else 
                result.append("\""+element+"\"");
                
        }
//        else if(f.is_string(&buf)){
//            std::cout<<"dentro\n";
//            result.append(buf);
//            result="\""+result+"\"";
//        }
        else if(!f.is_double(&d)){
            ss<<d;
            result.append(ss.str());
        }
    }
    //restituisci al stringa
    return result;
}

/**
 * 
 * @param FromEclipse
 * @param expNode
 * carica la definizione semantica restituita da ECLIPSE (FromEclipse)
 * istanziando i figli del nodo da espandere (expNode)
 */
void loadSemantics(EC_word FromEclipse,WM_node* expNode){
    double d;
    WM_node *newSon;
    EC_word head,tail,name,rtm,releaserList,s_tail,releaser;
    
        
    //estrai un sottoschema dalla lista di ECLIPSE
    FromEclipse.is_list(head,tail);  
    //estrai gli elementi del sottoschema
    head.is_list(name,s_tail);
    //s_tail.is_list(abstract,s_tail);
    s_tail.is_list(rtm,s_tail);
    s_tail.is_list(releaserList,s_tail);
    
    //aggiungi il nuovo nodo
//    if(name.is_string(s))
//        newSon=expNode->addSon("\""+functor2string(name)+"\"");
//    else
    
    
    
    newSon=expNode->addSon(functor2string(name));
    
    //std::cout<<"son added!\n";
    
    //std::cout<<"LOADING: "<<newSon->instance<<"\n";
    
    rtm.is_double(&d);
    //eredito l'amplificazione di mio padre
    newSon->amplification=expNode->amplification;
    
    //la magnitudine è il massimo tra la mia e quella di mio padre
//    if(expNode->magnitude>d)
//        newSon->magnitude=expNode->magnitude;
//    else
//        newSon->magnitude=d;
    newSon->ltMagnitude=d;
    
    //estrai i vari elementi del releaser
    while(releaserList.is_nil())
    {
        releaserList.is_list(releaser,releaserList);
        newSon->releaser.push_back(functor2string(releaser));
    }
    
    //carica nella WM ulteriori sottoschemi
    if(tail.is_nil())
        loadSemantics(tail, expNode);
}

void printWM (WM_node *node){
    for(int i=0;i<node->son.size();i++)
    {
        std::cout<<node->name<<"->"
                <<(node->son[i])->instance<<", "
                <<(node->son[i])->rtm<<", "
                <<WM->getInstanceMagnitude(node->son[i]->instance);
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
                std::cout << "\033[0;32m "<<node->son[i]->releaser[j]<<" \033[0m";
            else
                std::cout << "\033[0;31m ~"<<node->son[i]->releaser[j]<<" \033[0m";
            
            j++;
        }
        
        if(node->son[i]->goalStatus())
            std::cout<<" accomplished\n";
        else
            std::cout<<"\n";
        printWM(node->son[i]);
    }
}


/**
 * 
 * @param schemaInstance
 * @return vettore di stringhe rappresentanti i parametri dello schema
 * in versione prolog-like:
 * EG.
 *      vec[0]="nome schema"
 *      vec[1]="primo parametro"
 *      vec[2]="secondo parametro"
 *      etc.
 * 
 * eventuali parametri che siano essi stessi funtori vengono
 * restituiti ugualmente come elemento del vettore
 * EG.
 *      vec[i]="fun1(fun2(x,y),z)"
 * 
 */
std::vector<std::string> instance2vector(std::string schemaInstance){
    bool isAtom=true, isString=false;
    char c;
    std::string app;
    std::vector<std::string> result;
    std::stringstream ss(schemaInstance);
    int count=0;
    //leggi il primo carattere della stringa
    ss>>c;
    //mentre non sei a fine stringa
    while(!ss.eof())
    {
        //se il carattere è un doppio apice e non sono in una stringa
        if(c=='"' && !isString){
            //allora sono in una stringa 
            isString=true;
            //aggiungo l'apice
            app=app+c;
        }
        //se il carattere è un doppio apice e sono in una stringa
        else if(c=='"' && isString){
            //la stringa è finita
            isString=false;
            //aggiungo l'apice
            app=app+c;
            //aggiungila come elemento del funtore
            //result.push_back(app);
        }
        //mentre sono in una stringa
        else if(isString){
            //aggiungi il carattere senza controllarlo
            app=app+c;
        }
        //se sono un atomo ed il carattere letto è una parentesi aperta
        else if(c=='(' && isAtom){
            //non sono più un atomo
            isAtom=false;
            //inserisco il nome come primo elemento del vettore
            result.push_back(app);
            //pulisco la stringa d'appoggio
            app="";
            //salto la parentesi
//            ss>>c;
        }
        else if(c=='('){
            count++;
            app=app+c;
        }
        else if(c==')'&& count!=0){
            count--;
            app=app+c;
        }   
        //se il carattere letto non è una virgola
        else if(c!=',' || count!=0)
            //aggiungilo alla stringa d'appoggio
            app=app+c;
        //altrimenti (ie. il carattere è una virgola)
        else {
            //inserisci la stringa d'appoggio nel vettore risultato
            result.push_back(app);
            //pulisci la stringa d'appoggio
            app="";
            //ho saltato la virgola
        }
        //leggi il successivo carattere
        ss>>c;   
    }
    //se lo schema non ha parametri aggiungi il solo nome (vec[0])
    if(isAtom) result.push_back(app);
    //altrimenti aggiungi l'ultima stringa rimuovendo l'ultima parentesi
    else{
        app.erase(app.size()-1);
        result.push_back(app);
    }
    //ritorna il vettore calcolato
    return result;
}

/**
 * 
 * @param node
 * rimuove il sottoalbero radicato in node dalla Working Memory
 * se node è nullo la funzione non ha effetto
 */
void remove(WM_node *node){
    int i;
    //esci se il nodo passato è null
    if(node==NULL) return;
    
    //per ogni figlio
    for(i=0;i<node->son.size();i++)
        //deallocalo
        remove(node->son[i]);
    //se ho un padre, cancella la mia reference da esso
    if(node->father!=NULL){
        
        for(i=0;i<node->father->son.size();i++)
                if(node->father->son[i]==node){
                        node->father->son.erase(node->father->son.begin()+i);
                }
        //dealloca me stesso
        //std::cout<<node->instance<<" removed!\n";
        delete node;
    }
    //se non ho un padre... quindi sono la radice (alive)
    else{
        node->instance="zombie";
    }
    
}

/**
 * 
 * @param vec
 *  insertion sort di std::vec<WM_node *>,
 *      l'ordine è decrescente (3 2 1 0)
 * @return sorted vec
 */
std::vector<WM_node *> sortNodes(std::vector<WM_node *> vec){
    
    std::vector<WM_node *> sorted;
    
    //controlla che l'elemento non sia già presente 
    for(int i=0; i<vec.size(); i++){
        bool present=false;
        for(int j=0; j<sorted.size(); j++)
            if(vec[i]->instance == sorted[j]->instance)
                present=true;
                
        if(!present)
            sorted.push_back(vec[i]);
    }
                
    
    for (int i = 0; i < sorted.size(); i++) {
        WM_node *swap=sorted[i];
    	double magValue = WM->getInstanceMagnitude(sorted[i]->instance);
        double rtmValue = sorted[i]->rtm;
        int pos = i;
        while (pos > 0 && ( (  rtmValue!=0 && rtmValue < sorted[pos - 1]->rtm ) || ( 
                rtmValue == sorted[pos - 1]->rtm && 
                    magValue > WM->getInstanceMagnitude(sorted[pos - 1]->instance ) ) ) ) {      	  
            sorted[pos] = sorted[pos - 1];
            pos = pos - 1;
        }

        sorted[pos] = swap;
    }
    return sorted;
}

/**
 * 
 * @param fs velocità lineare del robot in metri/sec
 * @param ts velocità angolare in gradi/sec
 * 
 * La funzione somma le velocità a quelle presenti nella memoria
 */
//void engineSend(double fs,double ts,double rtm,std::string me){
//    //controlla se è già stato impartito un comando
//    std::vector< WM_node*> winnerNode=WM->getNodesByInstance(winner);
//    if(winner==me || winnerNode.size()==0){
//        //std::cout<<"winner1: "<<me<<"\n";
//        perception["engine.command"]=1;
//        perception["engine.ts"]=ts;
//        perception["engine.fs"]=fs; 
//    }
//    else if(winnerNode.size()!=0 && winnerNode[0]->rtm>rtm){
//        //std::cout<<"winner2: "<<me<<"\n";
//        winner=me;
//        perception["engine.command"]++;
//        perception["engine.ts"]=perception["engine.ts"]+ts;
//        perception["engine.fs"]=perception["engine.fs"]+fs;
//    }
//    
//    //std::cout<<"sended: "<<perception["engine.fs"]<<","<<perception["engine.ts"]<<"\n";
//}

/**
 * controllo della presenza di alive in WM
 * @return true se alive è stato rimosso, false altrimenti
 * 
 * la funzione rilascia il semaforo memMutex, va chiamata come segue:
 * 
 *      if(dead()) return;
 * 
 * in un qualsiasi ambito protetto.
 */
bool dead(){
    if(WM==NULL){
        pthread_mutex_unlock(&memMutex);
        return true;
    }
    return false;
}

void *execution(void *arg){
    //recast del parametro passato al thread
    Behaviour *behaviour=(Behaviour*)arg;
    
    //inizializzazione del behaviour
    behaviour->start();
    //mentre il behaviour è presente nella WM
    while(behaviour->perceptWM())
    {
        //se il suo releaser è attivo
        if (behaviour->perceptualSchema() && behaviour->getReleaser() && !behaviour->isFreezed()) {
            
            pthread_mutex_lock(&memMutex);
            behaviour->setNodeInternalReleaser(true);
            pthread_mutex_unlock(&memMutex);
            
            //avvia il comportamento
            behaviour->motorSchema();

            //se non è stato fatto l'update manuale del ritmo
            if (!behaviour->isUpdated())
                //settalo al valore di defaut (1/magnitude)
                behaviour->setDefaultRtm();
        }
        //se il releaser non è attivo setta il ritmo a quiescenza
        else {
            pthread_mutex_lock(&memMutex);
            //metti il ritmo a quiescenza, ie. perdi tutte le competizioni
            //behaviour->setRtm(QUIESCENCE);
            behaviour->setNodeInternalReleaser(false);
            pthread_mutex_unlock(&memMutex);
        }
        
        //attendi in base al ritmo
        
        unsigned int usec=(int) (behaviour->getRtm()*MSECOND);
        //unsigned int usec=(int) (0.1*MSECOND);
        //durante la sleep si rilascia la memoria di lavoro
        usleep(usec); 
    }
    //finalizzazione del behaviour
    behaviour->exit();
}
