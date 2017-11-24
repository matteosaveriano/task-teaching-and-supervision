/* 
 * File:   Param.h
 * Author: lorenzo
 *
 * Created on 7 dicembre 2012, 14.09
 */

#ifndef PARAM_H
#define	PARAM_H

#define ACTION_TIME 10
#define MAIN_MODE 0


const double LAMBDA=0.5;

//Train
const int N_POINT =200;
const double EPSILON=1.0/((double)N_POINT*30.0);

const int K_VAL=50;
const int ORIZZ=100;
const double DISC=0.9;

//Test
#define N_TEST 100

#endif	/* PARAM_H */

