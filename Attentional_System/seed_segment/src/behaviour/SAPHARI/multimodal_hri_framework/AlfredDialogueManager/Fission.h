/* 
 * File:   Fission.h
 * Author: lorenzo
 *
 * 
 * Modulo di Fissione: genera un'immagine per ogni azione macchina, e 
 * sintetizza l'audio
 * 
 * Created on 15 gennaio 2013, 12.34
 */

#ifndef SMILE_H
#define	SMILE_H
#include "QGraphicsScene"
#include <string>

class Fission : public QGraphicsScene {
    Q_OBJECT
    public:
    Fission();
    Fission(const Fission& orig);
    virtual ~Fission();
    /*http://harmattan-dev.nokia.com/docs/library/html/qt4/qgraphicsview.html*/
    QGraphicsView* view;
public slots:
    /*updateFission aggiorna l'immagine mostrata e genera una sintesi vocale.
     Il parametro passato è l'azione macchina che si sta eseguendo con 
     * eventuali informazioni separate da caratteri speciali come  "-" o ":". Queste informazioni
     * sono utili per costruire le frasi sulle richieste (scelta tra due etc.)
     *  .*/
    void updateFission(QString name);
private:
    /*Puntatore all'immagine mostrata.
     http://qt-project.org/doc/qt-4.8/qgraphicspixmapitem.html*/
    QGraphicsPixmapItem* item;
};

#endif	/* SMILE_H */

