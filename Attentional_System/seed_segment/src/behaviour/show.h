/* 
 * File:   show.h
 * Author: hargalaten
 *
 * Created on 17 aprile 2015, 10.46
 */

#include "../seed_header.h"
#include <graphviz/gvc.h>
#include <graphviz/cgraph.h>

#ifndef SHOW_H
#define	SHOW_H


class ShowBehaviour : public Behaviour {
public:
    ShowBehaviour(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        

      
        char * args[] = {
            "dot",
            "-Tpng", /*gif output*/
            "-Oabc1.png" /*output to file abc.gif*/
        };
        
        //prendi la lista dei nodi da visualizzare
        targetNodes=WM->getNodesByInstance(instance2vector(instance)[1]);
        
        /* set up a graphviz context */
        gvc = gvContext();
        
        /* parse command line args - minimally argv[0] sets layout engine */
        gvParseArgs(gvc, sizeof(args)/sizeof(char*), args);
        
        cv::namedWindow(this->getInstance(), cv::WINDOW_NORMAL); // Create a window for display.
        
    }
    bool perceptualSchema(){
        return true;
    }
    void motorSchema(){
        bool targetNotAlive=false;
        /* Create a simple digraph */
        graph = agopen("g", Agdirected, NULL);
        
        pthread_mutex_lock(&memMutex);

        //non amplificare questo processo
        WM->getNodesByInstance(this->getInstance())[0]->amplification=0;
        
        //setRtm(0.1);
        Agnode_t * alive;
        //se il nodo target non è ALIVE
        if(targetNodes[0]!=WM){
            targetNotAlive=true;

            //alloca un agNode alive da visualizzare
            alive=wmNode2agNode(WM);
        }

        //per ogni nodo dell'istanza da visualizzare
        for (int i = 0; i < targetNodes.size(); i++) {
            //crea un agNode root come radice del sottoalbero
            Agnode_t *root = wmNode2agNode(targetNodes[i]);
            //se il target è alive
            if(targetNotAlive){
                //aggiungi un edge da alive alla radice del sottoalbero
                Agedge_t *e = agedge(graph, alive, root, NIL(char*), TRUE);
                //setta l'edge come tratteggiato
                agsafeset(e,"style","dashed","");
            }
            //visualizza ricorsivamente il sottoalbero
            wm2graph(targetNodes[i], root);
        }
        pthread_mutex_unlock(&memMutex);
       
        /* Compute a layout using layout engine from command line args */
        gvLayoutJobs(gvc, graph);
        /* Write the graph according to -T and -o options */
        gvRenderJobs(gvc, graph);
        /* Free layout data */
        gvFreeLayout(gvc, graph);
        /* Free graph structures */
        agclose(graph);
        
        cv::Mat graphImg;
        //leggi il png generato da graphviz
        graphImg=cv::imread("noname.gv.png",CV_LOAD_IMAGE_COLOR);
        if (!graphImg.data) // Check for invalid input
        {
            std::cout << "Could not open or find the image" << std::endl;
            return;
        }
        
        cv::imshow(this->getInstance(), graphImg); // Show our image inside it.

        cv::waitKey(3); 
        
    }
    void wm2graph(WM_node *node, Agnode_t *f){
        //std::cout<<node->instance<<"\n";
        
        //per ogni nodo figlio
        for(int i=0; i<node->son.size(); i++){
            //se il nodo è stato espanso
            if (node->son[i]->expanded) {
                //crea un agNode che rappresenta il nodo attuale
                Agnode_t *n = wmNode2agNode(node->son[i]);
                //aggiungi un edge che lega il padre al nuovo nodo figlio
                Agedge_t *e = agedge(graph, f, n, NIL(char*), TRUE);

                //stampa gli elementi del releaser sull'arco
                showReleaserOnEdge(e,node->son[i]);

                //stampa il sottoalbero radicato nel figlio
                wm2graph(node->son[i], n);
            }
        }
        
    }
    Agnode_t *wmNode2agNode(WM_node* node){
        std::stringstream ss;

//        //KUKA-LEARNED-BEHAVIORS
//        if(node->name == "kukaTake" || node->name == "kukaPlace")
//            if(WMV.get<double>(node->instance + ".known")!=1)
//                ss<<"[learnable]\n";
        if(node->name == "kukaExecute"){
            //ss << "exe(" << instance2vector(node->instance)[1] << ","
            ss << instance2vector(node->instance)[1] << "("
                    << instance2vector(node->instance)[2] << ")\n"
                    << node->rtm << " ("
                    << WM->getInstanceMagnitude(node->instance) << ")";
        }
        else{
        //crea una stringa con nome e ritmo
        ss << node->instance << "\n" 
                << node->rtm << " ("
                << WM->getInstanceMagnitude(node->instance) << ")"; 
        }
        
        std::string str = ss.str();
        //trasforma la stringa in char*
        char * writable = new char[str.size() + 1];
        std::copy(str.begin(), str.end(), writable);
        writable[str.size()] = '\0';
        //crea un nuovo agNode
        Agnode_t *n = agnode(graph, writable, TRUE);

        delete[] writable;
        agsafeset(n, "penwidth", "2.0", "");
        if(node->abstract)
            agsafeset(n, "style", "dashed", "");
        
        //se il goal e risolto
        if(node->goalStatus())
            //setta il colore del nodo a blu
            agsafeset(n, "color", "blue", "");
        //se il nodo è attivo (ie. non quiescente e rilasciato)
        else if (node->rtm != QUIESCENCE && WM->isReleased(node->instance)) { //node->son[i]->isBranchReleased() ){
            //setta il colore del nodo a verde
            agsafeset(n, "color", "green", "");
        } //altrimenti
        else //se il colore del nodo non è verde (ie. non ho già disegnato altre istanze rilasciate)
            if (agget(n, "color") != "green" && agget(n, "color") != "blue")
                //setta il colore del nodo a rosso
                agsafeset(n, "color", "red", "");

        
        //ritorna il nodo creato
        return n;
    }
    void showReleaserOnEdge(Agedge_t *e, WM_node *n){
        bool nodeReleased = true;
        std::stringstream ss;
//        ss<<"<";
        for(size_t i=0; i<n->releaser.size(); i++){
            if(WMV.get<double>(n->releaser[i]) != 1){
                nodeReleased = false;
//                ss<<"<FONT COLOR=\"red\">";
                ss<<"~";
            }
//            else
//                ss<<"<FONT COLOR=\"green\">";

            if(instance2vector(n->releaser[i])[0] == "kukaExecute")
                ss<<instance2vector(n->releaser[i])[1]<<".done";
            else
                ss<<n->releaser[i];
//            ss<<"</FONT>";

            if(i != n->releaser.size()-1)
                ss<<"\n";
        }
//        ss<<">";

        std::string str = ss.str();
        //trasforma la stringa in char*
        char * writable = new char[str.size() + 1];
        std::copy(str.begin(), str.end(), writable);
        writable[str.size()] = '\0';

        agsafeset(e,"label",writable,"");
        delete[] writable;

        if(nodeReleased)
            agsafeset(e,"fontcolor","green","");
        else
            agsafeset(e,"fontcolor","red","");
    }

    void start() {

    }
    void exit(){
        //libera il contesto del graph
        gvFreeContext(gvc);
        
        // chiudi la finestra
        cv::destroyWindow(this->getInstance());
        cv::waitKey(1); 
    }
protected:
    GVC_t *gvc;
    Agraph_t *graph;
    std::vector< WM_node * > targetNodes;
};

#endif	/* SHOW_H */

